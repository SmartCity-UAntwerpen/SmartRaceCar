#!/usr/bin/sh
#
#	CONSTANTS
#
#	The URL to the most recent JAR file for the RacecarBackend
JAR_LOCATION="https://github.com/SmartCity-UAntwerpen/SmartRaceCar/raw/master/SmartRaceCar/Modules/racecarbackend/release/RacecarBackend.jar"

#	The URL of the most recent .properties file for the RacecarBackend
PROPERTIES_LOCATION="https://raw.githubusercontent.com/SmartCity-UAntwerpen/SmartRaceCar/master/SmartRaceCar/Modules/RacecarBackend.properties"

#	The (local) name of the JAR file
JAR_FILENAME="RacecarBackend.jar"

#	The (local) name of the .properties file
PROPERTIES_FILENAME="RacecarBackend.properties"

#	The (local) name of the log file created
LOG_FILENAME="RacecarBackend.log"

#	The command used to start the JAR
START_COMMAND="java -jar $JAR_FILENAME"

#
#	SCRIPT START
#
# If we got an argument (Any argument), pull new files from github
if [[ $# -gt 0 ]]
then
	#	Remove old JAR and .properties file
	rm ${JAR_FILENAME} ${PROPERTIES_FILENAME}

	#	Download new JAR and properties
	wget --no-cache -O ${JAR_FILENAME} ${JAR_LOCATION}
	wget --no-cache -O ${PROPERTIES_FILENAME} ${PROPERTIES_LOCATION}
fi

#	Find the Process ID of the old JAR
OLD_PID=$( pgrep -f "${START_COMMAND}" )

#	If the old process ID contained atleast 1 character (If the process wasn't running, we don't need to kill it), kill the old process
if [ ${#OLD_PID} -ge 1 ]
then
	kill $OLD_PID
fi

#	Check if the log file exists
if [ -f $LOG_FILENAME ]
then
	# Clean up the log file
	rm ${LOG_FILENAME}
fi

# Start the new process
nohup $START_COMMAND &