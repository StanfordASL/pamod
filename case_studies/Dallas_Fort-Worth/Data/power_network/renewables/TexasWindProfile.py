
# coding: utf-8

# In[1]:


# Load stuff
import datetime as dt
import pandas as pd
import geopandas as gpd
from shapely.geometry import Point
#import urllib
from subprocess import call
import glob
import os.path
import csv
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.axes3d import Axes3D
import pickle

import urllib.request, urllib.error, urllib.parse
from urllib.request import Request, urlopen
from urllib.error import URLError
# #import urllib
import os
from bs4 import BeautifulSoup

# %matplotlib inline


# In[2]:


#!jupyter nbconvert --to script TexasWindProfile.ipynb


# In[3]:


PLOT_FLAG = False
cwd = '/home/frossi2/Git/AMoD-power/atx'


# In[4]:


# A function to download zip files
#Author: Bellerophon_
#Date Created: 04/12/15
#Usage: Used to scrape a website for links that end in .zip and list them
#Requirements: BeautifulSoup lib
#Notes: 

def DownloadZipFiles(_url,_dlurl):
    #Create a new directory to put the files into
    #Get the current working directory and create a new directory in it named test
    #cwd = os.getcwd()
    dirname = 'TexasWind'
    newdir = cwd + '/'+ dirname
    print(("The current Working directory is " + cwd))
    newfile = open(newdir+'/zipfiles.txt','w')
    print(newfile)


    print("Downloading.. ")
    #Set variable for page to be open and url to be concatenated 
    url = _url
    dlurl = _dlurl
    page = urllib.request.urlopen(dlurl).read()

    #File extension to be looked for. 
    extension = ".zip"

    #Use BeautifulSoup to clean up the page
    soup = BeautifulSoup(page,"lxml")
    soup.prettify()

    #Find all the links on the page that end in .zip
    for anchor in soup.findAll('a', href=True):
        links = url + anchor['href']
        if True:#links.endswith(extension):
            newfile.write(links + '\n')
    newfile.close()

    #Read what is saved in zipfiles.txt and output it to the user
    #This is done to create presistent data 
    newfile = open(newdir+'/zipfiles.txt', 'r')
    #for line in newfile:
    #    print line
    newfile.close()

    #Read through the lines in the text file and download the zip files.
    #Handle exceptions and print exceptions to the console
    with open(newdir+'/zipfiles.txt', 'r') as url:
        for line in url:
            if line:
                try:
                    ziplink = line
                    #Removes the first 48 characters of the url to get the name of the file
                    #zipfile = line[48:]
                    #Removes the last 4 characters to remove the .zip
                    #zipfile2 = zipfile[:3]
                    #print("Trying to reach " + ziplink)
                    response = urllib.request.urlopen(ziplink)
                except URLError as e:
                    if hasattr(e, 'reason'):
                        print('We failed to reach a server.')
                        print(('Reason: ', e.reason))
                        continue
                    elif hasattr(e, 'code'):
                        print('The server couldn\'t fulfill the request.')
                        print(('Error code: ', e.code))
                        continue
                else:
                    zipcontent = response.read()
                    zipName = response.info()['Content-Disposition'] 
                    zipName=zipName[21:]
                    print(zipName)
                    completeName = os.path.join(newdir, zipName)
                    with open (completeName, 'w') as f:
                        print(("downloading.. " + zipName))
                        f.write(zipcontent)
                        f.close()
    print("Script completed")


# # Wind data

# In[5]:


# Download files
DownloadZipFiles('http://mis.ercot.com','http://mis.ercot.com/misapp/GetReports.do?reportTypeId=13028&reportTitle=Wind%20Power%20Production%20-%20Hourly%20Averaged%20Actual%20and%20Forecasted%20Values&showHTMLView=&mimicKey')


# In[6]:


# Data from http://mis.ercot.com/misapp/GetReports.do?reportTypeId=12311&reportTitle=Seven-Day%20Load%20Forecast%20by%20Forecast%20Zone&showHTMLView=&mimicKey
#  via http://www.ercot.com/gridinfo/generation

#Unzip all files that end in csv.zip
zipfilenamepattern = cwd+'/TexasWind/*WPP*_csv.zip'
zipfilenames = glob.glob(zipfilenamepattern)
unzip_folder = cwd+'/TexasWind/'
for zipfilename in zipfilenames:
    if not os.path.exists(zipfilename[:-8]+'.csv'): 
        call(['unzip','-n',zipfilename,'-d',unzip_folder])

csvfilenamepattern = cwd+'/TexasWind/cdr.*WPP*.csv'
csvfilenamepattern = glob.glob(csvfilenamepattern)
print((csvfilenamepattern[0], len(csvfilenamepattern)))


# In[7]:


#Read all CSVs in a Pandas notebook

HourlyWinds=pd.DataFrame()

for csvname in csvfilenamepattern:
    HourlyWind=pd.read_csv(csvname)
    HourlyWinds=HourlyWinds.append(HourlyWind,ignore_index=True)


# In[8]:


#Create an index for time 
HourlyWinds = HourlyWinds.dropna(axis=0,how='any',subset=['ACTUAL_SYSTEM_WIDE'])
HourlyWinds.HOUR_ENDING=HourlyWinds.HOUR_ENDING-1

SortedHourlyWinds=HourlyWinds.groupby(['DELIVERY_DATE','HOUR_ENDING']).mean()

AvgHourlyWinds=HourlyWinds.groupby(['HOUR_ENDING']).mean()
if PLOT_FLAG:
    plt.figure()
    plt.plot(SortedHourlyWinds['ACTUAL_SYSTEM_WIDE'].values)

    plt.figure()

    AvgHourlyWinds['ACTUAL_SYSTEM_WIDE'].plot()
    plt.figure()
    for myday in HourlyWinds['DELIVERY_DATE'].unique():
        daywinds=SortedHourlyWinds.loc[myday]
        plt.plot(daywinds['ACTUAL_SYSTEM_WIDE'].values)


# In[9]:



HourlyWinds['ts']=HourlyWinds.DELIVERY_DATE+' '+HourlyWinds.HOUR_ENDING.map(str)+':00'
HourlyWinds['ts']

HourlyWinds['ts']=pd.to_datetime(HourlyWinds['ts'])
HourlyWinds.set_index(['ts'],inplace=True)
#pd.to_datetime(HourlyWinds['DELIVERY_DATE'],HourlyWinds['HOUR_ENDING'])


# In[10]:


if PLOT_FLAG:
    HourlyWinds['ACTUAL_SYSTEM_WIDE'].plot()


# In[11]:


np.save(cwd+'/TexasAvgWindProfile.npy',AvgHourlyWinds['ACTUAL_SYSTEM_WIDE'])


# In[12]:


AvgHourlyWinds['ACTUAL_SYSTEM_WIDE']


# # Solar data

# In[13]:


#Download solar data
DownloadZipFiles('http://mis.ercot.com','http://mis.ercot.com/misapp/GetReports.do?reportTypeId=13483&reportTitle=Solar%20Power%20Production%20-%20Hourly%20Averaged%20Actual%20and%20Forecasted%20Values&showHTMLView=&mimicKey')


# In[14]:



#Extract
zipfilenamepattern = cwd+'/TexasWind/cdr.*.PVG*_csv.zip'
zipfilenames = glob.glob(zipfilenamepattern)
for zipfilename in zipfilenames:
    if not os.path.exists(zipfilename[:-8]+'.csv'): 
        call(['unzip','-n',zipfilename,'-d',unzip_folder])

csvfilenamepattern = cwd+'/TexasWind/cdr.*PVG*.csv'
csvfilenamepattern = glob.glob(csvfilenamepattern)
print((csvfilenamepattern[0], len(csvfilenamepattern)))


# In[15]:


#Read all CSVs in a Pandas notebook

HourlySun=pd.DataFrame()
for csvname in csvfilenamepattern:
    #print csvname
    HourlySunT=pd.read_csv(csvname)
    HourlySun=HourlySun.append(HourlySunT,ignore_index=True)


# In[16]:


#Create an index for time 
HourlySun = HourlySun.dropna(axis=0,how='any',subset=['ACTUAL_SYSTEM_WIDE'])
HourlySun.HOUR_ENDING=HourlySun.HOUR_ENDING-1

SortedHourlySun=HourlySun.groupby(['DELIVERY_DATE','HOUR_ENDING']).mean()
AvgHourlySun=HourlySun.groupby(['HOUR_ENDING']).mean()

if PLOT_FLAG:
    plt.figure()
    plt.plot(SortedHourlySun['ACTUAL_SYSTEM_WIDE'].values)

    plt.figure()
    AvgHourlySun['ACTUAL_SYSTEM_WIDE'].plot()
    plt.figure()
    for myday in HourlySun['DELIVERY_DATE'].unique():
        daysun=SortedHourlySun.loc[myday]
        plt.plot(daysun['ACTUAL_SYSTEM_WIDE'].values)


# In[17]:



HourlySun['ts']=HourlySun.DELIVERY_DATE+' '+HourlySun.HOUR_ENDING.map(str)+':00'
HourlySun['ts']

HourlySun['ts']=pd.to_datetime(HourlySun['ts'])
HourlySun.set_index(['ts'],inplace=True)


# In[18]:


if PLOT_FLAG:
    HourlySun['ACTUAL_SYSTEM_WIDE'].plot()


# In[19]:


np.save(cwd+'/TexasAvgSunProfile.npy',AvgHourlySun['ACTUAL_SYSTEM_WIDE'])


# ## Load Data by weather region

# In[20]:


DownloadZipFiles('http://mis.ercot.com','http://mis.ercot.com/misapp/GetReports.do?reportTypeId=13101&reportTitle=Actual%20System%20Load%20by%20Weather%20Zone&showHTMLView=&mimicKey')


# In[21]:


# Data from http://mis.ercot.com/misapp/GetReports.do?reportTypeId=12311&reportTitle=Seven-Day%20Load%20Forecast%20by%20Forecast%20Zone&showHTMLView=&mimicKey
#  via http://www.ercot.com/gridinfo/generation

#Unzip all files that end in csv.zip
zipfilenamepattern = cwd+'/TexasWind/*ACTUALSYSLOAD*_csv.zip'
zipfilenames = glob.glob(zipfilenamepattern)
unzip_folder = cwd+'/TexasWind/'
for zipfilename in zipfilenames:
    if not os.path.exists(zipfilename[:-8]+'.csv'): 
        call(['unzip','-n',zipfilename,'-d',unzip_folder])

csvfilenamepattern = cwd+'/TexasWind/cdr.*ACTUALSYSLOAD*.csv'
csvfilenamepattern = glob.glob(csvfilenamepattern)
print((csvfilenamepattern[0], len(csvfilenamepattern)))


# In[22]:


#Read all CSVs in a Pandas notebook

HourlyLoad=pd.DataFrame()
for csvname in csvfilenamepattern:
    #print csvname
    HourlyLoadT=pd.read_csv(csvname)
    HourlyLoad=HourlyLoad.append(HourlyLoadT,ignore_index=True)


# In[23]:


#Create an index for time 
HourlyLoad = HourlyLoad.dropna(axis=0,how='any',subset=['TOTAL'])
HourlyLoad['HourEndingStr']=HourlyLoad['HourEnding']
def HourStrToNum(_hstr):
    return int(_hstr[:2])
HourlyLoad.HourEnding=HourlyLoad.HourEndingStr.apply(HourStrToNum)


# In[24]:


regions = ['COAST','EAST','FAR_WEST','NORTH','NORTH_C','SOUTHERN','SOUTH_C','WEST']

HourlyLoad.HourEnding=HourlyLoad.HourEnding-1

SortedHourlyLoad=HourlyLoad.groupby(['OperDay','HourEnding']).mean()
AvgHourlyLoad=HourlyLoad.groupby(['HourEnding']).mean()

if PLOT_FLAG:
    plt.figure()
    plt.plot(SortedHourlyLoad['TOTAL'].values)

    plt.figure()
    AvgHourlyLoad['TOTAL'].plot()
    plt.figure()
    for myday in HourlyLoad['OperDay'].unique():
        dayload=SortedHourlyLoad.loc[myday]
        plt.plot(dayload['TOTAL'].values)
    plt.figure()
    for myregion in regions:
        plt.plot(AvgHourlyLoad[myregion].values)


# In[25]:



HourlyLoad['ts']=HourlyLoad.OperDay+' '+HourlyLoad.HourEnding.map(str)+':00'
HourlyLoad['ts']

HourlyLoad['ts']=pd.to_datetime(HourlyLoad['ts'])
HourlyLoad.set_index(['ts'],inplace=True)


# In[26]:


if PLOT_FLAG:
    HourlyLoad.plot()


# In[27]:


#Pickle road graph
output = open(cwd+'/LoadsByRegion.pkl', 'wb')
pickle.dump(AvgHourlyLoad, output)
output.close()


