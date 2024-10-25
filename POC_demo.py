#RFID TAG ID's
left = ""
right = ""

print("STARTING...")

while True:
    reading = input() #scan RFID tag

    if reading == left:
        print("move right")
    elif reading == right:
        print("move left")
    else:
        print("haven't programmed this tag as left or right yet!")
