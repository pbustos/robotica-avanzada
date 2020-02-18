#! /usr/bin/env python3

#import pyhton
import sys
import os
import time
import threading

#import api kortex
from kortex_api.autogen.client_stubs.BaseClientRpc import BaseClient
from kortex_api.autogen.client_stubs.BaseCyclicClientRpc import BaseCyclicClient
from kortex_api.autogen.messages import Base_pb2, BaseCyclic_pb2, Common_pb2

#import librerias creadas
import basicMovements as bM

if __name__ == "__main__":
        
    # Import the utilities helper module
    sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
    import utilities

    # Parse arguments
    args = utilities.parseConnectionArguments()
    
    # Create connection to the device and get the router
    with utilities.DeviceConnection.createTcpConnection(args) as router:

        # Create required services
        base = BaseClient(router)
        base_cyclic = BaseCyclicClient(router)

        # Example core
        success = True

        success &= bM.cartesian_Home_movement(base, base_cyclic)
        print("Move Home \n")

        #success &= bM.cartesian_Especific_movement(base, base_cyclic, 0.57, 0.0, 0.43, 90.0, 0, 90)
        #print("Move Home Robot KINOVA\n")

        #success &= bM.cartesian_Relative_movement(base, base_cyclic, 0, 0, 0, 0, 0, -10)
        #print("Girar mano \n")
        
        success &= bM.gripper_Close_All(base)
        print("Close All \n")

        success &= bM.gripper_Open_All(base)
        print("Open All \n")

        success &= bM.gripper_Relative_Aperture(base, 0.50)
        print("Open/close 0.50 \n")

        if success:
            print("Movimientos completados \n")
        else:
            print("Se han producido errores en los movientos \n")