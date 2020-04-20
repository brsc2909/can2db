# can2db
set of tools for sending can data to different databases

### Installing
```bash
sudo apt-get install libhiredis-dev

git clone git@github.com:brsc2909/can2db.git

cd can2db

mkdir build
cd build && cmake ..

make

sudo make install
```

### can2redis
useable in basic form, needs tuning

### can2atlas
still needs a lot of work, not really usable. 

### can2mysql
not implemented yet

### can2influx

### can2