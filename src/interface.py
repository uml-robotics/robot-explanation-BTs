import os

print "Test Explainable BT interface: \nEnter \"1\" for \"What are you doing?\"\nEnter \"2\" for \"Why are you doing this?\" "

while True:
    q = int(raw_input("Question type: "))
    if q == 1:
        what = "What are you doing?"
    elif q == 2:
        what = "Why are you doing this?"
    print what
    os.system("rosservice call /explainable_bt \"what: '" + what + "'\"")
