#!/usr/bin/python
#This Python script generates a list of source files.
#It finds the correct VPATH to each source file from make, and then feeds all info to gcc -MM.
import sys
import os
from os import path, getenv

# first argument is the gcc compiler
# thereafter are the vpaths closed by a '+'
# thereafter are the srcs closed by a '+'
# thereafter are the cflags

#parse the input arguments into seperate lists
cc = sys.argv[1]
vpaths = list()
srcs = list()
cflags = list()
mode = 0
for i in range(2,len(sys.argv)):
    vpath = sys.argv[i]
    if vpath == '+':
        mode = mode + 1
    else:
        if mode == 0:
            vpaths.append(vpath)
        elif mode == 1:
            srcs.append(sys.argv[i])
        elif mode == 2:
            cflags.append(sys.argv[i])

#create the command for gcc -MM cflags srcs
cmd = cc + ' -MM '
for flg in cflags: 
    #remove some flags that appeared here for unknown reasons since last time I made this
    id1 =flg.find("-MD -MP -MF")
    if (id1 > -1):        
        id2 =flg.find("!") 
        flg = flg[0:id1] + flg[id2+1:]

    #append the vpaths to includes that for some reason are now in the CFLAGS
    #they are in the form -Dsomestuff-Isomepath -Ianotherpath ...
    prev=0
    tmpflg = ""
    while (prev < len(flg)):
        flgss = flg[prev:]  
        #print("Old; " + flgss)      
        
        id1 =flgss.find(" -I")

        if (id1 > -1):
            
            flgsst = flgss[id1+3:]
            id2 =flgsst.find(" ")
            
            if (id2 == -1):
                id2 = len(flgsst)
            else:
                id2 = id2
            
            flgsst = flgss[id1+3:id1+3+id2] #extract an include from the flg subsstring string
            #print("Found: " + flgsst)
            found = 0
            iflgss = ""
            for vpath in vpaths:                
                if os.path.isdir(path.join(vpath, flgsst)) :
                    iflgss = "-I" + path.join(vpath, flgsst) + ' '
                    found = found +1                    
                    break
            if found == 0:                
                iflgss = "ERROR, could not find: " + flgsst
                print(iflgss)
            if found > 1:
                iflgss = "ERROR, found more than once: " + iflgss
                print(iflgss)



            #replace the original include from the flags
            flg = flg[:prev+id1] + ' ' + iflgss +  flg[prev+id1+id2+3:]
            #print("new: " + flg)
            prev=prev+id1+len(iflgss) #skip to the next character after the upgraded include in the original string


        else:
            prev = len(flg)+1


    qflg = "" # append all quotes with escape char
    for i in range(0,len(flg)):
        if flg[i] == "\"":
            qflg = qflg +  "\\\""
        else:
            qflg = qflg + flg[i]
    cmd = cmd + qflg + ' '

for i in range(0,len(srcs)):
    f = srcs[i]
    found = 0
    for vpath in vpaths:
        if os.path.isfile(path.join(vpath, f)) :
            cmd = cmd + path.join(vpath, f) + ' '
            found = found +1
            break
    if found == 0:
        tmps = "ERROR, could not find: " + f
        cmd = cmd + tmps #hack to make the error known
        print(tmps)
    if found > 1:
        tmps = "ERROR, found more than once: " + f
        cmd = cmd + tmps #hack to make the error known
        print(tmps)

#call gcc
os.system(cmd)

