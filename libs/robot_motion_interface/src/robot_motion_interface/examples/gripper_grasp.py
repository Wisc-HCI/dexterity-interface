from robot_motion_interface.tesollo.tesollo_communicaton import TesolloCommunication
# TODO: Interact through interface instead


def main():
    """
    Simple example to open and close gripper
    """
    print("About to connect to Tesollo")
    com = TesolloCommunication()
    com.connect("192.168.4.8", "502") 
    print("Just connected to Tesollo")

    print(com.get_position())


    com.disconnect()
    # Right tesollo: 192.168.4.7


if __name__ == "__main__":
    main()
