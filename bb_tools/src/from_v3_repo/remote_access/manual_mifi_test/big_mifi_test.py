import remote_access
import settings


SELECTED_PORT = settings.BIG_PC_PORT

def main():

    connection = remote_access.RemoteAccess(SELECTED_PORT)
    connection.run() 

if __name__ == "__main__":
    main()
