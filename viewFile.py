import pickle


filename = 'dataStore'
file = open(filename, 'rb')
dataRead = []
while 1:
    try:
        dataRead.append(pickle.load(file)) #this loads each frame in one at a time, this is where we can process each frame with the transformation matrix
    except EOFError:
        break
