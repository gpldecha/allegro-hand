#  Makefile
#
#  ~~~~~~~~~~~~
#
#  PCANBasic JNI Interface
#
#  ~~~~~~~~~~~~
#
#  ------------------------------------------------------------------
#  Author : Thomas Haber (thomas@toem.de)
#  Last change: 18.06.2010
#  Language: make
#  ------------------------------------------------------------------
#
#  Copyright (C) 1999-2010  PEAK-System Technik GmbH, Darmstadt
#  more Info at http://www.peak-system.com
#  ------------------------------------------------------------------
# linux@peak-system.com
# www.peak-system.com
#
# This program is free software; you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation; either version 2 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program; if not, write to the Free Software
# Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
#
#****************************************************************************

#JAVA_HOME = /usr/lib/jvm/java-6-sun/
#JAVA_HOME = /usr/lib/jvm/java-7-openjdk-i386
JAVA_HOME = /usr/lib/jvm/java-7-openjdk-amd64

SRC     = src
INC     = -I. -I../pcanbasic -I$(JAVA_HOME)/include/ -I$(JAVA_HOME)/include/linux
FILES   = $(SRC)/pcanbasic_jni.c
DBG     = -g
RT      = NO_RT

$(info JAVA_HOME is $(JAVA_HOME))


ifeq ($(HOSTTYPE),x86_64)
  LIBPATH = /usr/lib64
else
  LIBPATH = /usr/lib
endif

LDNAME  = libPCANBasic_JNI.so
SONAME  = $(LDNAME).0
TARGET  = $(SONAME).1

CFLAGS = -fPIC -shared -O2 -Wall -Wl,-soname,$(SONAME) -lc $(INC) -D$(RT)


all:    $(TARGET)

$(TARGET) : $(FILES) 
	$(CC) $(FILES) $(CFLAGS) -o $(TARGET) -lpcanbasic
	ln -sf $(TARGET) $(LDNAME)

clean:
	rm -f $(SRC)/*~ $(SRC)/*.o *~ *.so.* *.so

#********** this entry is reserved for root access only *********************
install:
	cp $(TARGET) $(LIBPATH)/$(TARGET)
	ln -sf $(LIBPATH)/$(TARGET) $(LIBPATH)/$(SONAME)
	ln -sf $(LIBPATH)/$(SONAME) $(LIBPATH)/$(LDNAME)
	/sbin/ldconfig
  
uninstall:
	rm $(LIBPATH)/$(TARGET)
	rm $(LIBPATH)/$(SONAME)
	rm $(LIBPATH)/$(LDNAME)
	/sbin/ldconfig
  

