#nom de l'executable
EXEC = quad_pilot

#source : rech de .cpp
SRC=$(wildcard *.cpp)
#construction des objets
OBJ=$(SRC:.cpp=.o)

#-----------------------------------------------------------
#compilation :
CC = g++
LIBS= -lm -lrt -lwiringPi #-llapack

#option de debugage :
OPT_Debug= -O2 -Wall

# Set DMP FIFO rate to 20Hz to avoid overflows on 3d demo.  See comments in
# MPU6050_6Axis_MotionApps20.h for details.
# F = 200/(DDMP_FIFO_RATE + 1)
#the program is not fast enough to read DMP at 200Hz => DDMP_FIFO_RATE = 1

CXXFLAGS = -DDMP_FIFO_RATE=1


ALL:$(EXEC)

$(EXEC):$(OBJ)
	$(CC) $(CXXFLAGS) $(OPT_Debug) -o $@ $^ $(LIBS)

%.o: %.cpp
	$(CC) $(CXXFLAGS) $(OPT_Debug) -o $@ -c $^ $(LIBS)

install: quad_pilot
	[ "`id -u`" = "0" ] || { echo "Must be run as root"; exit 1; }
	cp -f quad_pilot /usr/local/sbin
	cp -f init-script /etc/init.d/quad_pilot
	chmod 755 /etc/init.d/quad_pilot
	update-rc.d quad_pilot defaults

uninstall:
	[ "`id -u`" = "0" ] || { echo "Must be run as root"; exit 1; }
	[ -e /etc/init.d/quad_pilot ] && /etc/init.d/quad_pilot stop || :
	update-rc.d quad_pilot remove
	rm -f /usr/local/sbin/quad_pilot
	rm -f /etc/init.d/quad_pilot
clean:
	rm -rf *.o *~ *.mod
	rm -rf $(EXEC)
