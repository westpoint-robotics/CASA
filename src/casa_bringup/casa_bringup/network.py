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
        msg = d.split('(')
        ip = msg[1][:-2]
        name = msg[0].split(" ")[-2]
        
        if name in saved_names and ip != my_ip:
            ids.append(int(ip.split('.')[-1]))

    return ids


def myIP(inerface_id: str):
    return ni.ifaddresses(interface_id)[ni.AF_INET][0]['addr']
    
