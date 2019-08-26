import requests
from tkinter import *

def click1():
  latco= lat.get()
  longco= long.get()
  url= ("https://api.mapbox.com/styles/v1/mapbox/streets-v11/static/"+longco + "," + latco + ",14.25,0,60/600x600?access_token=pk.eyJ1IjoibWFjaGV0ZTg5IiwiYSI6ImNqeXh3ZDd1YTEzZWwzY21sdGx2bnhwM3gifQ.MbW6tttXkNk1p24wviwHQw")  # Gets a map at longco, latco
                                                                                                                                                                                                                               # zoom 14.24, bearing 0, and pitch 60
                                                                                                                                                                                                                               # 600 pixels wide and 600 pixels high map is obtained
                                                                                                                                                                                                                               # access_token is obtained after creating a mapbox account
                                                                                                                                                                                                                               
  r=requests.get(url)
  with open('test.jpeg', 'wb') as f:
    f.write(r.content) 


window = Tk()
window.title (" Project ")
window.geometry("350x250")

Label (window, text = "Enter the lat (-90 to 90)", font='calibri 10').place(relx = 0.3, rely = 0.3, anchor = CENTER)
lat = Entry(window, width =20, bg= 'white')
lat.place(relx = 0.7, rely = 0.3, anchor = CENTER)

Label(window, text="Enter the long (-180 to 180)", font='calibri 10').place(relx = 0.3, rely = 0.5, anchor = CENTER)
long= Entry(window, width= 20, bg= 'white')
long.place(relx = 0.7, rely = 0.5, anchor = CENTER)

Button(window, text=("create map 1"), width=10, command =click1).place(relx = 0.5, rely = 0.65, anchor = CENTER)
Button(window, text=("create map 2"), width=10, ).place(relx = 0.5, rely = 0.8, anchor = CENTER)  #insert command= <function name>


window.mainloop()


