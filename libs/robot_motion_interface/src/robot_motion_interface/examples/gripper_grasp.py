from robot_motion_interface.tesollo.tesollo_communicaton import TesolloCommunication
# TODO: Interact through interface instead


import time

def main():
    """
    Simple example to open and close gripper
    """
    print("About to connect to Tesollo")
    com = TesolloCommunication()
    com.connect("192.168.4.8", "502") 
        # Right tesollo: 192.168.4.7
    print("Just connected to Tesollo")
    time.sleep(1)

    print("TRYING TO GET POSITION")
    print(com.get_position())


    com.disconnect()



if __name__ == "__main__":
    main()
