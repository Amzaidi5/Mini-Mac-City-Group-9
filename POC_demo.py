#RFID TAG ID's
left = "E280F302000000008927897C"
right = "E280F3020000000089278990"

print("STARTING...")

while True:
    reading = input() #scan RFID tag

    if reading == left:
        print("move right")
    elif reading == right:
        print("move left")
    else:
        print("haven't programmed this tag as left or right yet!")
