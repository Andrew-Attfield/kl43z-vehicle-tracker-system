
#!/bin/bash


#variable assignment
LOG_FILE="SpeedLog.log"
DB_FILE="Database.txt"


#create database.txt if it doesnt exist
if [ ! -f "$DB_File" ];
then
	touch "$DB_FILE"
fi

while true;
do

	#get ID's
	LAST_LOG_ID=$(tail -n 1 "$LOG_FILE" | cut -d: -f1)
	LAST_DB_ID=$(tail -n 1 "$DB_FILE" 2>/dev/null | cut -d -f1) #?

	#default 0 if empty
	LAST_DB_ID=${LAST_DB_ID:-0}

	#check new records
	if [ "$LAST_LOG_ID" -gt "$LAST_DB_ID" ];
	then
		#read new
		NEW_RECORDS=$(sed -n "$((LAST_DB_ID + 1), $LAST_LOG_ID p" "$LOG_FILE")

		while IFS= read -r line
		do

			ID=$(echo "$line" | cut -d -f1)
			SPEED=$(echo "$line" | cut -d: -f2)
			SPEEDLIMIT=$(echo "$line" | cut -d: -f3)

			#calc speed delta
			DELTA=$((SPEED - SPEEDLIMIT))

			#determine fine
			if [ "$DELTA" -lt 10 ];
			then
				FINE=0
			elif [ "$DELTA" -lt 20 ];
			then
				FINE=50
			elif [ "$DELTA" - lt 30];
			then
				FINE=100
			else
				FINE=$((100 + 10 * (DELTA - 30)))
			fi

		#append to database.txt
		echo "$ID:$SPEED:$SPEEDLIMIT:$FINE" >> "$DB_FILE"

	done <<< "$NEW_RECORDS"
	fi

	#wait 10 seconds for next loop
	sleep 10

done





