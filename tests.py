import time

now = time.time()
while now + 10 > time.time():
    print("miau")
    time.sleep(0.2)