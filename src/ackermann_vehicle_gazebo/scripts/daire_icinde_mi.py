#!/usr/bin/env python
# -*-coding: utf-8 -*-

import math

def main():
    x = float(input("Noktanın x değerini gir : "))
    y = float(input("Noktanın y değerini gir : ")) 


    cember_merkez_x = float(input("Çember merkez x değerini gir : "))
    cember_merkez_y = float(input("Çember merkez y değerini gir : "))
    yaricap = float(input("Çember yariçap  değerini gir : "))

    print("\niçinde mi : " + str(icinde_mi(x, y, cember_merkez_x, cember_merkez_y, yaricap)))


def icinde_mi(a, b, c, d, e = 1):
    f = pow(a-c, 2) + pow(b-d, 2)
    mesafe = pow(f, 0.5)

    if(mesafe <= e):
        return 1 # içindedir 
    else:
        return 0 # dışındadır

while(1):
    main()