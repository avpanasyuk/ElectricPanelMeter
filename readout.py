# -*- coding: utf-8 -*-
"""
Created on Sat Feb  3 16:33:52 2018

@author: panasyuk
"""

import requests
import time

while True:
  try:
    response = requests.get('http://192.168.2.169/read')
  except ConnectionError:
    continue
  t = response.text
  open_ss = '<html>\r\n'
  close_ss = '<br></html>'
  t = t[t.find(open_ss) ++ len(open_ss):t.find(close_ss)]
  t = t.replace('<br>',',')
  print(t)
  with open('/RAIDZ2/R/PowerMon1.txt', 'a') as file:
    file.write(str(time.time()) + ',' + t + '\n')
  time.sleep(5)
  
  
