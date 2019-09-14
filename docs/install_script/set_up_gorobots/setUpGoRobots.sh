#!/bin/bash

# check wether user has got admin rights
#if [ "$(whoami)" != "root" ];
#then
   #echo "You will need to be root to execute this script "
   #echo "Please contact your admin"
   #echo 
   #exit
#fi

#remember directory to later go back to copy project files
OWN_DIR=$( (cd -P $(dirname $0) && pwd) )


# Installing required packages
echo '----------------- Required Packages -----------------'
echo 
read -p 'Do you need to install the required packages - You need to be root to do so? [y/n]: ' answer
if [ $answer = "y" ];
then
	echo '----installing required packages-------------------------------'
	echo "sudo apt-get install g++ make m4 libreadline-dev libgsl0-dev libglu-dev libgl1-mesa-dev libopenscenegraph-dev libqt4-dev qt4-qmake libqt4-qt3support openjdk-6-jdk automake gnuplot libgsl0ldbl xutils-dev libltdl-dev libtool subversion eclipse-platform eclipse-pde"
	sudo apt-get install g++ make m4 libreadline-dev libgsl0-dev libglu-dev libgl1-mesa-dev libopenscenegraph-dev libqt4-dev qt4-qmake libqt4-qt3support openjdk-6-jdk automake gnuplot libgsl0ldbl xutils-dev libltdl-dev libtool subversion eclipse-platform eclipse-pde
	echo "sudo apt-get install freeglut3 freeglut3-dev"
	sudo apt-get install freeglut3 freeglut3-dev
	echo "sudo apt-get install binutils-gold"
	sudo apt-get install binutils-gold
	echo "sudo apt-get install git libncurses-dev"
	sudo apt-get install git libncurses-dev

	read -p 'Do you want to install drivers for the Hokuyo Laser Range Finder? [y/n]: ' answer
	if [ $answer = "y" ];
	then
		sudo apt-get install liburg0-dev
	fi

	read -p 'Do you want to install drivers for the xIMU Inertial Measurement Unit? [y/n]: ' answer
	if [ $answer = "y" ];
	then
		#Installing driver for IMU USB sensor----Start
		echo "sudo apt-get install libusb-1.0-0"
		sudo apt-get install libusb-1.0-0
		echo "Install FTDI drivers"
		#Download drivers and extract them
		wget -q "http://www.ftdichip.com/Drivers/D2XX/Linux/libftd2xx1.1.12.tar.gz"
		mkdir ftdi
		tar -xf libftd2xx1.1.12.tar.gz -C ftdi
		#Check for 64 Bit and copy corresponding libraries (hide output of cp)
		if grep -wq lm /proc/cpuinfo;
		then
			sudo cp ftdi/release/build/x86_64/lib* /usr/local/lib 2>&1 | grep -v 'omitting directory'
		else
			sudo cp ftdi/release/build/i386/lib* /usr/local/lib 2>&1 | grep -v 'omitting directory'
		fi
		#Make library accessible and create symbolic links
		sudo chmod 0755 /usr/local/lib/libftd2xx.so.1.1.12
		sudo ln -sf /usr/local/lib/libftd2xx.so.1.1.12 /usr/local/lib/libftd2xx.so
		#Remove downloaded files
		rm -r ftdi libftd2xx1.1.12.tar.gz
		echo "Done"
		#Installing driver for IMU USB sensor----End
	fi	

	echo 
	echo
else
	echo "Skipped installing required packages according to your input"
	echo "Continue "
	echo
fi

# Attaching important settings to the .bashrc
echo '----------------- Checking your .bashrc -----------------'
echo
read -p "Do you want to include the settings to your .bashrc? [y/n]: " answer
echo 
if [ $answer = "y" ];
then
	## Include the settings to the .bashrc
	echo '----adapt your .bashrc-----------------------------------------'
	echo "adding ${HOME}/include ${HOME}/lib and ${HOME}/bin to respective PATH variable "
	cd $HOME
	#check for correct path to osg...
	PFAD=`cd /usr/lib/; ls | grep osgPlug`
	echo ' ' >> .bashrc
	echo '# definitions for lpzrobots' >> .bashrc
	echo  'export CPATH="$HOME/include"' >> .bashrc
	echo  'export LIBRARY_PATH="$HOME/lib"' >> .bashrc
	echo  'export LD_LIBRARY_PATH=${LD_LIBRARY_PATH}:$HOME/lib:/usr/lib/'$PFAD >> .bashrc
	echo  'export PATH=${PATH}:$HOME/bin' >> .bashrc

	echo "reload .bashrc"
	echo "source ${HOME}/.bashrc"
	source ${HOME}/.bashrc
else
	echo "Skipped attaching settings to .bashrc according to your input"
	echo "Continue "
	echo
fi

if [ ! -d $HOME/workspace/ ];
then 
	mkdir $HOME/workspace/;
 fi


# Fetching the files from the repositories
echo '----------------- Fetching source files -----------------'
echo
read -p 'Do you want to fetch the files from git repositories and copy the project settings file? [y/n] : ' answer
echo 
if [ $answer = "y" ];
then

	echo
	echo "For fetching the files of the GIT repository, this script needs the address of your two GIT-repositories"
	echo "For example: crauterb-lpzrobots-fork"
	read -p "What is the name of your LpzRobots-fork? : " LPZadress
	read -p "What is the name of your GoRobots-fork? : " GOadress
	echo
	echo
	echo '----fetch source files from git repositories------------------'
	# neccesary, if eclipse was freshly installed and not yet opened
	
	cd ${HOME}/workspace

	echo "assembla.com username: "
	read USERNAME
	echo
	echo "cloning gorobots repository"
	echo "git clone https://${USERNAME}@git.assembla.com/${GOadress}.git -b master"
	git clone https://${USERNAME}@git.assembla.com/${GOadress}.git -b master

	echo
	echo "cloning lpzrobots repository"
	echo "git clone https://${USERNAME}@git.assembla.com/${LPZadress}.git -b master"
	git clone https://${USERNAME}@git.assembla.com/${LPZadress}.git -b master
	echo 
	echo
	echo '----copy project settings to lpzrobots and gorobots folder----'
	cd ${OWN_DIR}
	echo "cp lpzproject ${HOME}/workspace/${LPZadress}/.project"
	echo "cp lpzcproject ${HOME}/workspace/${LPZadress}/.cproject"
	echo "cp goproject ${HOME}/workspace/${GOadress}/.project"
	echo "cp gocproject ${HOME}/workspace/${GOadress}/.cproject"
	cp lpzproject ${HOME}/workspace/${LPZadress}/.project
	cp lpzcproject ${HOME}/workspace/${LPZadress}/.cproject
	cp goproject ${HOME}/workspace/${GOadress}/.project
	cp gocproject ${HOME}/workspace/${GOadress}/.cproject

else
	echo "Not fetching source files according to your input"
	echo "Continue "
	echo
fi


# Compiling the files via the make-file
echo '----------------- Compile LpzRobots -----------------'
echo
read -p 'Do you want to compile LpzRobots now? [y/n] : ' answer
echo 
if [ $answer = "y" ];
then
	#echo `pwd`
	LPZadress=`ls $HOME/workspace | grep lpz`
	echo 
	echo
	echo '----make LpzRobots -------------------------------------------'
	echo "cd ${HOME}/workspace/${LPZadress}"
	echo "make all"	
	cd ${HOME}/workspace/${LPZadress}
	make all

else
	echo "Not compiling LpzRobots according to your input"
	echo "Continue "
	echo
fi

echo 
echo "----------------- Finished Setup -----------------"
echo " Exit"
echo 
exit 0
