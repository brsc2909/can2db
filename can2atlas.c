#define _POSIX_C_SOURCE 200809L
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <unistd.h>
#include <string.h>
#include <netdb.h>
#include <signal.h>
#include <ctype.h>
#include <libgen.h>
#include <errno.h>
#include <sys/time.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <sys/uio.h>
#include <netinet/in.h>
#include <net/if.h>

#include <linux/can.h>
#include <linux/can/raw.h>
#include <can-utils/lib.h>

#include <time.h>
#include <math.h>


#define ARRAY_SIZE(arr) (sizeof(arr) / sizeof((arr)[0]))

/* for hardware timestamps - since Linux 2.6.30 */
#ifndef SO_TIMESTAMPING
#define SO_TIMESTAMPING 37
#endif

/* from #include <linux/net_tstamp.h> - since Linux 2.6.30 */
#define SOF_TIMESTAMPING_SOFTWARE (1<<4)
#define SOF_TIMESTAMPING_RX_SOFTWARE (1<<3)
#define SOF_TIMESTAMPING_RAW_HARDWARE (1<<6)

#define MAXSOCK 16    /* max. number of CAN interfaces given on the cmdline */
#define MAXIFNAMES 30 /* size of receive name index to omit ioctls */
#define ANYDEV "any"  /* name of interface to receive from any CAN interface */
#define ANL "\r\n"    /* newline in ASC mode */

#define ATLAS_DEFAULT_HOST "localhost"
#define ATLAS_DEFAULT_PORT 7101


static char *cmdlinename[MAXSOCK];
static __u32 dropcnt[MAXSOCK];
static __u32 last_dropcnt[MAXSOCK];
static char devname[MAXIFNAMES][IFNAMSIZ+1];
static int  dindex[MAXIFNAMES];
static int  max_devname_len; /* to prevent frazzled device name output */ 
const int canfd_on = 1;

#define MAXANI 4
const char anichar[MAXANI] = {'|', '/', '-', '\\'};
const char extra_m_info[4][4] = {"- -", "B -", "- E", "B E"};

extern int optind, opterr, optopt;

static volatile int running = 1;

void print_usage(char *prg)
{
	fprintf(stderr, "\nUsage: %s -h <hostname> -p <port> [options] <CAN interface>+\n", prg);
	fprintf(stderr, "  (use CTRL-C to terminate %s)\n\n", prg);
	fprintf(stderr, "Options: -h <hostname>  (hostname or ip of atlas server (Default: %s))\n", ATLAS_DEFAULT_HOST);
	fprintf(stderr, "         -p <port>      (atlas server port (default: %i)\n", ATLAS_DEFAULT_PORT);
	fprintf(stderr, "         -r <size>   (set socket receive buffer to <size>)\n");
	fprintf(stderr, "         -D          (Don't exit if a \"detected\" can device goes down.\n");
	fprintf(stderr, "         -d          (monitor dropped CAN frames)\n");
	fprintf(stderr, "         -T <msecs>  (terminate after <msecs> without any reception)\n");
	fprintf(stderr, "\n");
	fprintf(stderr, "Up to %d CAN interfaces with optional filter sets can be specified\n", MAXSOCK);
	fprintf(stderr, "on the commandline in the form: <ifname>[,filter]*\n");
	fprintf(stderr, "\nComma separated filters can be specified for each given CAN interface:\n");
	fprintf(stderr, " <can_id>:<can_mask> (matches when <received_can_id> & mask == can_id & mask)\n");
	fprintf(stderr, " <can_id>~<can_mask> (matches when <received_can_id> & mask != can_id & mask)\n");
	fprintf(stderr, " #<error_mask>       (set error frame filter, see include/linux/can/error.h)\n");
	fprintf(stderr, " [j|J]               (join the given CAN filters - logical AND semantic)\n");
	fprintf(stderr, "\nCAN IDs, masks and data content are given and expected in hexadecimal values.\n");
	fprintf(stderr, "When the can_id is 8 digits long the CAN_EFF_FLAG is set for 29 bit EFF format.\n");
	fprintf(stderr, "Without any given filter all data frames are received ('0:0' default filter).\n");
	fprintf(stderr, "\nUse interface name '%s' to receive from all CAN interfaces.\n", ANYDEV);
	fprintf(stderr, "\nExamples:\n");
	fprintf(stderr, "%s -c -c -ta can0,123:7FF,400:700,#000000FF can2,400~7F0 can3 can8\n", prg);
	fprintf(stderr, "%s -l any,0~0,#FFFFFFFF    (log only error frames but no(!) data frames)\n", prg);
	fprintf(stderr, "%s -l any,0:0,#FFFFFFFF    (log error frames and also all data frames)\n", prg);
	fprintf(stderr, "%s vcan2,12345678:DFFFFFFF (match only for extended CAN ID 12345678)\n", prg);
	fprintf(stderr, "%s vcan2,123:7FF (matches CAN ID 123 - including EFF and RTR frames)\n", prg);
	fprintf(stderr, "%s vcan2,123:C00007FF (matches CAN ID 123 - only SFF and non-RTR frames)\n", prg);
	fprintf(stderr, "\n");
}

void sigterm(int signo)
{
	running = 0;
}

int idx2dindex(int ifidx, int socket) {

	int i;
	struct ifreq ifr;

	for (i=0; i < MAXIFNAMES; i++) {
		if (dindex[i] == ifidx)
			return i;
	}

	/* create new interface index cache entry */

	/* remove index cache zombies first */
	for (i=0; i < MAXIFNAMES; i++) {
		if (dindex[i]) {
			ifr.ifr_ifindex = dindex[i];
			if (ioctl(socket, SIOCGIFNAME, &ifr) < 0)
				dindex[i] = 0;
		}
	}

	for (i=0; i < MAXIFNAMES; i++)
		if (!dindex[i]) /* free entry */
			break;

	if (i == MAXIFNAMES) {
		fprintf(stderr, "Interface index cache only supports %d interfaces.\n",
		       MAXIFNAMES);
		exit(1);
	}

	dindex[i] = ifidx;

	ifr.ifr_ifindex = ifidx;
	if (ioctl(socket, SIOCGIFNAME, &ifr) < 0)
		perror("SIOCGIFNAME");

	if (max_devname_len < strlen(ifr.ifr_name))
		max_devname_len = strlen(ifr.ifr_name);

	strcpy(devname[i], ifr.ifr_name);

#ifdef DEBUG
	printf("new index %d (%s)\n", i, devname[i]);
#endif

	return i;
}

void error(const char *msg) { perror(msg); exit(0); }

int64_t get_timestamp(void)
{

    struct timespec spec;

    clock_gettime(CLOCK_REALTIME, &spec);

    return (spec.tv_sec*1000)+(spec.tv_nsec/1e6); // Convert nanoseconds to milliseconds
}

void prepare_request(char *req_message, char *host, char *endpoint, int payload_size,  struct canfd_frame canframe){
    /*
    POST /api/v1/publish HTTP/1.0
    Content-Type: application/json
    Host: hostname
    Content-Length: payload_size
    */
    char *msg = req_message;
	int64_t timestamp = get_timestamp();

    char *hostname = "localhost";

    sprintf(msg, "POST %s HTTP/1.0\r\n", endpoint);
    sprintf(msg + strlen(msg), "Content-Type: application/json\r\n");
    sprintf(msg + strlen(msg), "Host: %s\r\n", host);
    sprintf(msg + strlen(msg), "Content-Length: %i   \r\n\r\n", payload_size);

	int g = strlen(msg);

    sprintf(msg + strlen(msg), "{ \"tags\": { \"nf.node\": \"%s\" }, \"metrics\": [ ", hostname);
    for (__u8 i = 0; i < canframe.len; i++)
    {	
        sprintf(msg+strlen(msg),"{ \"tags\": { \"name\": \"%X\", \"can.data_pos\": %i, \"atlas.dstype\": \"gauge\" }, \"timestamp\": %lu, \"value\": %X },", 
            canframe.can_id, 
            i, 
            timestamp, 
            canframe.data[i]);
    }
    sprintf(msg+strlen(msg)-1,"]}\r\n");
	g = strlen(msg) - g;
	
	if (g != payload_size){
		payload_size = g;
		g = sprintf(msg, "POST %s HTTP/1.0\r\n", endpoint);
		g += sprintf(msg + g, "Content-Type: application/json\r\n");
		g += sprintf(msg + g, "Host: %s\r\n", host);
		g += sprintf(msg + g, "Content-Length: %i   \r\n\r\n", payload_size);

		g += sprintf(msg + g, "{ \"tags\": { \"nf.node\": \"%s\" }, \"metrics\": [ ", hostname);
    	for (__u8 i = 0; i < canframe.len; i++)
    	{	
        	g += sprintf(msg + g,"{ \"tags\": { \"name\": \"%X\", \"can.data_pos\": %i, \"atlas.dstype\": \"gauge\" }, \"timestamp\": %lu, \"value\": %X },", 
					canframe.can_id, 
					i, 
					timestamp, 
					canframe.data[i]);
		}
		g += sprintf(msg+g-1,"]}\r\n");
	}


}


void send_to_atlas(struct sockaddr_in serv_addr, char * host ,char *path, struct canfd_frame canframe){
    int sockfd, bytes, sent, received, total;
    char *message, response[4096];

    

    /* create the socket */
    sockfd = socket(AF_INET, SOCK_STREAM, 0);
    if (sockfd < 0) error("ERROR opening socket");

    /* connect the socket */
    if (connect(sockfd, &serv_addr,sizeof(serv_addr)) < 0)
        error("ERROR connecting");

    // HERE we need to prepare the message
	int http_header = 103;
    int metric_pl_length = 112+(8);
    int payload_size = 44+strlen(host)+(metric_pl_length*canframe.len);
    message = malloc(http_header + payload_size);

    

    prepare_request(message, host, path, payload_size, canframe);



    /* send the request */
    total = strlen(message);
    sent = 0;
    do {
        bytes = write(sockfd,message+sent,total-sent);
        if (bytes < 0)
            error("ERROR writing message to socket");
        if (bytes == 0)
            break;
        sent+=bytes;
    } while (sent < total);



    /* receive the response */
    memset(response,0,sizeof(response));
    total = sizeof(response)-1;
    received = 0;
    do {
        bytes = read(sockfd,response+received,total-received);
        if (bytes < 0)
            error("ERROR reading response from socket");
        if (bytes == 0)
            break;
        received+=bytes;
    } while (received < total);

    if (received == total){
        error("ERROR storing complete response from socket");
	}

    /* close the socket */
    close(sockfd);

    free(message);

}

int main(int argc,char *argv[])
{
    /* first where are we going to send it? */
    int port = ATLAS_DEFAULT_PORT;
    char *hostname = ATLAS_DEFAULT_HOST;
    char *path = "/api/v1/publish";

    struct hostent *server;
    struct sockaddr_in serv_addr;


    /* CANBUS variables */
    fd_set rdfs;
	int s[MAXSOCK];
	unsigned char timestamp = 0;
	unsigned char hwtimestamp = 0;
	unsigned char down_causes_exit = 1;
	unsigned char dropmonitor = 0;
	unsigned char view = 0;
	int count = 0;
	int rcvbuf_size = 0;
	int opt, ret;
	int currmax, numfilter;
	int join_filter;
	char *ptr, *nptr;
	struct sockaddr_can addr;
	char ctrlmsg[CMSG_SPACE(sizeof(struct timeval) + 3*sizeof(struct timespec) + sizeof(__u32))];
	struct iovec iov;
	struct msghdr msg;
	struct cmsghdr *cmsg;
	struct can_filter *rfilter;
	can_err_mask_t err_mask;
	struct canfd_frame frame;
	int nbytes, i;
	struct ifreq ifr;
	struct timeval tv;
	struct timeval timeout, timeout_config = { 0, 0 }, *timeout_current = NULL;

    signal(SIGTERM, sigterm);
	signal(SIGHUP, sigterm);
	signal(SIGINT, sigterm);

	while ((opt = getopt(argc, argv, "t:h:p:HciaSlDdxLn:r:heT:?")) != -1) {
		switch (opt) {
		case 't':
			timestamp = optarg[0];
			if ((timestamp != 'a') && (timestamp != 'A') &&
			    (timestamp != 'd') && (timestamp != 'z')) {
				fprintf(stderr, "%s: unknown timestamp mode '%c' - ignored\n",
				       basename(argv[0]), optarg[0]);
				timestamp = 0;
			}
			break;
		case 'h':
            hostname = optarg;
			break;

		case 'p':
			port = atoi(optarg);
			if (count < 1) {
				print_usage(basename(argv[0]));
				exit(1);
			}
			break;

		case 'D':
			down_causes_exit = 0;
			break;

		case 'd':
			dropmonitor = 1;
			break;

		case 'r':
			rcvbuf_size = atoi(optarg);
			if (rcvbuf_size < 1) {
				print_usage(basename(argv[0]));
				exit(1);
			}
			break;

		case 'T':
			errno = 0;
			timeout_config.tv_usec = strtol(optarg, NULL, 0);
			if (errno != 0) {
				print_usage(basename(argv[0]));
				exit(1);
			}
			timeout_config.tv_sec = timeout_config.tv_usec / 1000;
			timeout_config.tv_usec = (timeout_config.tv_usec % 1000) * 1000;
			timeout_current = &timeout;
			break;
		default:
			print_usage(basename(argv[0]));
			exit(1);
			break;
		}
	}

	if (optind == argc) {
		print_usage(basename(argv[0]));
		exit(0);
	}

	currmax = argc - optind; /* find real number of CAN devices */

	if (currmax > MAXSOCK) {
		fprintf(stderr, "More than %d CAN devices given on commandline!\n", MAXSOCK);
		return 1;
	}

    /* lookup the ip address */
    server = gethostbyname(hostname);
    if (server == NULL) error("ERROR, no such host");

    /* fill in the structure */
    memset(&serv_addr,0,sizeof(serv_addr));
    serv_addr.sin_family = AF_INET;
    serv_addr.sin_port = htons(port);
    memcpy(&serv_addr.sin_addr.s_addr,server->h_addr_list[0],server->h_length);

    	for (i=0; i < currmax; i++) {

		ptr = argv[optind+i];
		nptr = strchr(ptr, ',');

#ifdef DEBUG
		printf("open %d '%s'.\n", i, ptr);
#endif

		s[i] = socket(PF_CAN, SOCK_RAW, CAN_RAW);
		if (s[i] < 0) {
			perror("socket");
			return 1;
		}

		cmdlinename[i] = ptr; /* save pointer to cmdline name of this socket */

		if (nptr)
			nbytes = nptr - ptr;  /* interface name is up the first ',' */
		else
			nbytes = strlen(ptr); /* no ',' found => no filter definitions */

		if (nbytes >= IFNAMSIZ) {
			fprintf(stderr, "name of CAN device '%s' is too long!\n", ptr);
			return 1;
		}

		if (nbytes > max_devname_len)
			max_devname_len = nbytes; /* for nice printing */

		addr.can_family = AF_CAN;

		memset(&ifr.ifr_name, 0, sizeof(ifr.ifr_name));
		strncpy(ifr.ifr_name, ptr, nbytes);

#ifdef DEBUG
		printf("using interface name '%s'.\n", ifr.ifr_name);
#endif

		if (strcmp(ANYDEV, ifr.ifr_name)) {
			if (ioctl(s[i], SIOCGIFINDEX, &ifr) < 0) {
				perror("SIOCGIFINDEX");
				exit(1);
			}
			addr.can_ifindex = ifr.ifr_ifindex;
		} else
			addr.can_ifindex = 0; /* any can interface */

		if (nptr) {

			/* found a ',' after the interface name => check for filters */

			/* determine number of filters to alloc the filter space */
			numfilter = 0;
			ptr = nptr;
			while (ptr) {
				numfilter++;
				ptr++; /* hop behind the ',' */
				ptr = strchr(ptr, ','); /* exit condition */
			}

			rfilter = malloc(sizeof(struct can_filter) * numfilter);
			if (!rfilter) {
				fprintf(stderr, "Failed to create filter space!\n");
				return 1;
			}

			numfilter = 0;
			err_mask = 0;
			join_filter = 0;

			while (nptr) {

				ptr = nptr+1; /* hop behind the ',' */
				nptr = strchr(ptr, ','); /* update exit condition */

				if (sscanf(ptr, "%x:%x",
					   &rfilter[numfilter].can_id, 
					   &rfilter[numfilter].can_mask) == 2) {
 					rfilter[numfilter].can_mask &= ~CAN_ERR_FLAG;
					if (*(ptr+8) == ':')
						rfilter[numfilter].can_id |= CAN_EFF_FLAG;
					numfilter++;
				} else if (sscanf(ptr, "%x~%x",
						  &rfilter[numfilter].can_id, 
						  &rfilter[numfilter].can_mask) == 2) {
 					rfilter[numfilter].can_id |= CAN_INV_FILTER;
 					rfilter[numfilter].can_mask &= ~CAN_ERR_FLAG;
					if (*(ptr+8) == '~')
						rfilter[numfilter].can_id |= CAN_EFF_FLAG;
					numfilter++;
				} else if (*ptr == 'j' || *ptr == 'J') {
					join_filter = 1;
				} else if (sscanf(ptr, "#%x", &err_mask) != 1) { 
					fprintf(stderr, "Error in filter option parsing: '%s'\n", ptr);
					return 1;
				}
			}

			if (err_mask)
				setsockopt(s[i], SOL_CAN_RAW, CAN_RAW_ERR_FILTER,
					   &err_mask, sizeof(err_mask));

			if (join_filter && setsockopt(s[i], SOL_CAN_RAW, CAN_RAW_JOIN_FILTERS,
						      &join_filter, sizeof(join_filter)) < 0) {
				perror("setsockopt CAN_RAW_JOIN_FILTERS not supported by your Linux Kernel");
				return 1;
			}

			if (numfilter)
				setsockopt(s[i], SOL_CAN_RAW, CAN_RAW_FILTER,
					   rfilter, numfilter * sizeof(struct can_filter));

			free(rfilter);

		} /* if (nptr) */

		/* try to switch the socket into CAN FD mode */
		setsockopt(s[i], SOL_CAN_RAW, CAN_RAW_FD_FRAMES, &canfd_on, sizeof(canfd_on));

		if (rcvbuf_size) {

			int curr_rcvbuf_size;
			socklen_t curr_rcvbuf_size_len = sizeof(curr_rcvbuf_size);

			/* try SO_RCVBUFFORCE first, if we run with CAP_NET_ADMIN */
			if (setsockopt(s[i], SOL_SOCKET, SO_RCVBUFFORCE,
				       &rcvbuf_size, sizeof(rcvbuf_size)) < 0) {
#ifdef DEBUG
				printf("SO_RCVBUFFORCE failed so try SO_RCVBUF ...\n");
#endif
				if (setsockopt(s[i], SOL_SOCKET, SO_RCVBUF,
					       &rcvbuf_size, sizeof(rcvbuf_size)) < 0) {
					perror("setsockopt SO_RCVBUF");
					return 1;
				}

				if (getsockopt(s[i], SOL_SOCKET, SO_RCVBUF,
					       &curr_rcvbuf_size, &curr_rcvbuf_size_len) < 0) {
					perror("getsockopt SO_RCVBUF");
					return 1;
				}

				/* Only print a warning the first time we detect the adjustment */
				/* n.b.: The wanted size is doubled in Linux in net/sore/sock.c */
				if (!i && curr_rcvbuf_size < rcvbuf_size*2)
					fprintf(stderr, "The socket receive buffer size was "
						"adjusted due to /proc/sys/net/core/rmem_max.\n");
			}
		}

		if (timestamp) {

			if (hwtimestamp) {
				const int timestamping_flags = (SOF_TIMESTAMPING_SOFTWARE | \
								SOF_TIMESTAMPING_RX_SOFTWARE | \
								SOF_TIMESTAMPING_RAW_HARDWARE);

				if (setsockopt(s[i], SOL_SOCKET, SO_TIMESTAMPING,
						&timestamping_flags, sizeof(timestamping_flags)) < 0) {
					perror("setsockopt SO_TIMESTAMPING is not supported by your Linux kernel");
					return 1;
				}
			} else {
				const int timestamp_on = 1;

				if (setsockopt(s[i], SOL_SOCKET, SO_TIMESTAMP,
					       &timestamp_on, sizeof(timestamp_on)) < 0) {
					perror("setsockopt SO_TIMESTAMP");
					return 1;
				}
			}
		}

		if (dropmonitor) {

			const int dropmonitor_on = 1;

			if (setsockopt(s[i], SOL_SOCKET, SO_RXQ_OVFL,
				       &dropmonitor_on, sizeof(dropmonitor_on)) < 0) {
				perror("setsockopt SO_RXQ_OVFL not supported by your Linux Kernel");
				return 1;
			}
		}

		if (bind(s[i], (struct sockaddr *)&addr, sizeof(addr)) < 0) {
			perror("bind");
			return 1;
		}
	}
    // END Configure can device

	/* these settings are static and can be held out of the hot path */
	iov.iov_base = &frame;
	msg.msg_name = &addr;
	msg.msg_iov = &iov;
	msg.msg_iovlen = 1;
	msg.msg_control = &ctrlmsg;

	while (running) {

		FD_ZERO(&rdfs);
		for (i=0; i<currmax; i++)
			FD_SET(s[i], &rdfs);

		if (timeout_current)
			*timeout_current = timeout_config;

		if ((ret = select(s[currmax-1]+1, &rdfs, NULL, NULL, timeout_current)) <= 0) {
			//perror("select");
			running = 0;
			continue;
		}

		for (i=0; i<currmax; i++) {  /* check all CAN RAW sockets */

			if (FD_ISSET(s[i], &rdfs)) {

				int idx;

				/* these settings may be modified by recvmsg() */
				iov.iov_len = sizeof(frame);
				msg.msg_namelen = sizeof(addr);
				msg.msg_controllen = sizeof(ctrlmsg);  
				msg.msg_flags = 0;

				nbytes = recvmsg(s[i], &msg, 0);
				idx = idx2dindex(addr.can_ifindex, s[i]);

				if (nbytes < 0) {
					if ((errno == ENETDOWN) && !down_causes_exit) {
						fprintf(stderr, "%s: interface down\n", devname[idx]);
						continue;
					}
					perror("read");
					return 1;
				}

				if ((size_t)nbytes != CAN_MTU && (size_t)nbytes != CANFD_MTU){
					fprintf(stderr, "read: incomplete CAN frame\n");
					return 1;
				}

				if (count && (--count == 0))
					running = 0;
		    
				for (cmsg = CMSG_FIRSTHDR(&msg);
				     cmsg && (cmsg->cmsg_level == SOL_SOCKET);
				     cmsg = CMSG_NXTHDR(&msg,cmsg)) {
					if (cmsg->cmsg_type == SO_TIMESTAMP) {
						memcpy(&tv, CMSG_DATA(cmsg), sizeof(tv));
					} else if (cmsg->cmsg_type == SO_TIMESTAMPING) {

						struct timespec *stamp = (struct timespec *)CMSG_DATA(cmsg);

						/*
						 * stamp[0] is the software timestamp
						 * stamp[1] is deprecated
						 * stamp[2] is the raw hardware timestamp
						 * See chapter 2.1.2 Receive timestamps in
						 * linux/Documentation/networking/timestamping.txt
						 */
						tv.tv_sec = stamp[2].tv_sec;
						tv.tv_usec = stamp[2].tv_nsec/1000;
					} else if (cmsg->cmsg_type == SO_RXQ_OVFL)
						memcpy(&dropcnt[i], CMSG_DATA(cmsg), sizeof(__u32));
				}

				/* check for (unlikely) dropped frames on this specific socket */
				if (dropcnt[i] != last_dropcnt[i]) {

					__u32 frames = dropcnt[i] - last_dropcnt[i];

                    printf("DROPCOUNT: dropped %d CAN frame%s on '%s' socket (total drops %d)\n",
						       frames, (frames > 1)?"s":"", devname[idx], dropcnt[i]);

					last_dropcnt[i] = dropcnt[i];
				}

				/* once we detected a EFF frame indent SFF frames accordingly */
				if (frame.can_id & CAN_EFF_FLAG)
					view |= CANLIB_VIEW_INDENT_SFF;

                send_to_atlas(serv_addr,hostname,path, frame);
			}

			fflush(stdout);
		}
	}

    /* process response */
    return 0;
}