#!/usr/bin/env python

import csv
import os
class ubicacion_data:
    def __init__(self,dic):
        #ubicacion,pos_x,pos_y,pos_z,orientacion_x,orientacion_y,orientacion_z,orientacion_w
        self.ubicacion=dic["ubicacion"]
        self.px=dic["pos_x"]
        self.py=dic["pos_y"]
        self.pz=dic["pos_z"]
        
        self.ox=dic["orientacion_x"]
        self.oy=dic["orientacion_y"]
        self.oz=dic["orientacion_z"]
        self.ow=dic["orientacion_w"]
        
    def __repr__(self):
        return self.px

memoria_path=os.path.realpath(__file__).split("/")
memoria_path[-1]='memoria/ubicaiones.csv'
memoria_path="/".join(memoria_path)
key_s=[]
lista_val_od=[]
dict_orientacion={}
with open(memoria_path, newline='') as csvfile:
    reader = csv.DictReader(csvfile)
    #print(reader)
    
    for row in reader:
        print(row)
        key_s=list(row.keys())
        lista_val_od.append(row)
        dict_orientacion[row["ubicacion"]]=ubicacion_data(row)
        #print(row['first_name'], row['last_name'])
print(dict_orientacion)
memoria_path=os.path.realpath(__file__).split("/")
memoria_path[-1]='memoria/p.csv'
memoria_path="/".join(memoria_path)
diccionario_escribit={}

with open(memoria_path, 'w', newline='') as csvfile:
    fieldnames = key_s
    writer = csv.DictWriter(csvfile, fieldnames=fieldnames)
    
    writer.writeheader()
    for j in lista_val_od:
        writer.writerow(j)
    for i in range(3):
        diccionario_escribit={}
        for key_new in fieldnames:
            diccionario_escribit[key_new]=str(i)
        writer.writerow(diccionario_escribit)