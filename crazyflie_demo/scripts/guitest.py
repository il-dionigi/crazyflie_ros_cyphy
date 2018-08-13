import Tkinter as tk
from threading import Thread
import rospy

counter = 0 
clearMode = 0
def count():
    global counter
    counter += 1
    label.config(text=str(counter))
    
def counter_label(label):
    counter = 0
    count()
 
def show_entry_fields():
    global labelx, labely, labelyaw
    posMode = posSetting.get()
    if (posMode):
        labelx.config(text="x")
        labely.config(text="y")
        labelyaw.config(text="yaw")
        print("Using position mode")
        print("x: %s\ny: %s\nz: %s\nyaw: %s\n" % (e1.get(), e2.get(), e3.get(), e4.get() ))
    else:
        labelx.config(text="vx")
        labely.config(text="vy")
        labelyaw.config(text="yawrate")
        print("Using velocity mode")
        print("vx: %s\nvy: %s\nz: %s\nyawrate: %s\n" % (e1.get(), e2.get(), e3.get(), e4.get() ))
    if (clearMode):
        e1.delete(0, tk.END)
        e2.delete(0, tk.END)
        e3.delete(0, tk.END)
        e4.delete(0, tk.END)

def addC():
    global counter
    while(True):
        counter = counter + 1
        rospy.sleep(1)
        label.config(text=str(counter))

def setModePos():
    posMode = posSetting.get()
    if (posMode):
        labelx.config(text="x")
        labely.config(text="y")
        labelyaw.config(text="yaw")
        print("Using position mode")
    else:
        labelx.config(text="vx")
        labely.config(text="vy")
        labelyaw.config(text="yawrate")
        print("Using velocity mode")
 
def setModeClear():
    clearMode = clearSetting.get()
    if (clearMode):
        print("Text will be cleared")
    else:
        print("Text will stay")

def land():
    print("Landing..")

root = tk.Tk()
root.title("Setpoint")
setpointRow = 6
settingsRow = 0
modeRow = 1
posSetting = tk.IntVar()
clearSetting = tk.IntVar()

labelSettings = tk.Label(root, text="Settings")
labelSettings.grid(row=settingsRow, columnspan=2)

tk.Label(root, 
        text="""Choose the mode:""",
        justify = tk.LEFT,
        padx = 20).grid(row=modeRow)
tk.Radiobutton(root, 
              text="Position",
              padx = 20, 
              variable=posSetting, 
              value=1).grid(sticky=tk.W)
tk.Radiobutton(root, 
              text="Velocity",
              padx = 20, 
              variable=posSetting, 
              value=0).grid(sticky=tk.W)

tk.Button(root, text='Set Mode', command=setModePos, height=2).grid(row=modeRow+3, column=0)

tk.Label(root, 
        text="""Clear text after send:""",
        justify = tk.RIGHT,
        padx = 20).grid(row=modeRow, column=1)
tk.Radiobutton(root, 
              text="Yes",
              padx = 20, 
              variable=clearSetting, 
              value=1).grid(row=modeRow+1, column=1)
tk.Radiobutton(root, 
              text="No",
              padx = 20, 
              variable=clearSetting, 
              value=0).grid(row=modeRow+2, column=1)

tk.Button(root, text='Set Mode', command=setModeClear, height=2).grid(row=modeRow+3, column=1)


landButton = tk.Button(root, text='STOP / LAND', command=land, width=25)
landButton.grid(row=setpointRow-1, columnspan=2, padx=4)
landButton.grid_rowconfigure(1, weight=2)

labelx = tk.Label(root, text="x/vx")
labelx.grid(row=setpointRow)
labely = tk.Label(root, text="y/vy")
labely.grid(row=setpointRow+1)
tk.Label(root, text="z").grid(row=setpointRow+2)
labelyaw = tk.Label(root, text="yaw/yawrate")
labelyaw.grid(row=setpointRow+3)

e1 = tk.Entry(root)
e2 = tk.Entry(root)
e3 = tk.Entry(root)
e4 = tk.Entry(root)

e1.grid(row=setpointRow, column=1, padx=10)
e2.grid(row=setpointRow+1, column=1)
e3.grid(row=setpointRow+2, column=1)
e4.grid(row=setpointRow+3, column=1)


tk.Button(root, text='Quit', command=root.quit).grid(row=setpointRow+5, column=0, sticky=tk.W, pady=4)
tk.Button(root, text='Send', command=show_entry_fields).grid(row=setpointRow+5, column=1, sticky=tk.W, pady=4)
label = tk.Label(root, fg="dark green")
'''label.grid(row=4, column=1)
counter_label(label)
button = tk.Button(root, text='Count Up', width=25, command=count)
button.grid(row=4, column=0)'''

#countIt = Thread(target=addC)
#countIt.start()
root.mainloop( )
#countIt.join()
