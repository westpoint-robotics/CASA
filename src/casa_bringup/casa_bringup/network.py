"""
Author: Jason Hughes
Date: July 2023
About: Function to scan the network and return ip addresses of whats on the network
"""
import os
import netifaces as ni

ids = list()

def scan(saved_names: tuple, interface_id: str) -> list:

    my_ip = ni.ifaddresses(interface_id)[ni.AF_INET][0]['addr']
    
    for d in os.popen('nmap -sn 192.168.1.0/24', 'r', 1):
        print(d)
<<<<<<< HEAD
        msg_ind = d.find("192")
        msg = d[msg_ind:]
        if ")" in msg:
            ip = msg[:msg.find(")")]
        else:
            ip = msg

        if "(" in d:
            msg = d.split("(")
            name = msg[0].split(" ")[-2]
        else:
            name = saved_names[0]
            
=======
        msg = d.split('(')
        ip = msg[1][:-2]
        name = msg[0].split(" ")[-2]
        
>>>>>>> 91aa8540e7073830bdcb4ed2354d4dad794e6b99
        if name in saved_names and ip != my_ip:
            ag_id = int(ip.split('.')[-1])
            if ag_id > 232:
                ids.append(ag_id-100)
            else:
                ids.append(ag_id)

    return ids


def myIP(interface_id: str):
    return ni.ifaddresses(interface_id)[ni.AF_INET][0]['addr']
    
