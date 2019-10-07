def messenger(ser, setFieldID, setSelfID, currentlyMove):

        message = ser.read(19)
        if len(message)>17:
            message = str(message[5:17])
            fieldID = str(message[3])
            selfID = str(message[4])
            print(message)

            if fieldID == setFieldID and (selfID == setSelfID or selfID == "X"):
                if "START" in message:
                    ser.write(str.encode('rf:a' + fieldID + setSelfID + 'ACK----- \r \n'))
                    print("f")
                    return True
                elif "STOP" in message:
                    ser.write(str.encode('rf:a' + fieldID + setSelfID + 'ACK----- \r \n'))
                    print("Stop!")
                    return False
                elif "PING" in message:
                    print("DOS")
                    ser.write(str.encode('rf:a' + fieldID + setSelfID + 'ACK----- \r \n'))
                    return currentlyMove
                else:
                    return currentlyMove
            else:
                return currentlyMove
        else:
            return currentlyMove

