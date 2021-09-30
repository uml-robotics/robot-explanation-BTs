import os

print("Test Explainable BT interface: \nEnter \"1\" for \"What are you doing?\"\nEnter \"2\" for \"Why are you doing this?\"\nEnter \"3\" for \"How do you achieve your goal?\"")

while True:
    q = int(input("Question type: "))
    if q == 1:
        what = "What are you doing?"
    elif q == 2:
        what = "Why are you doing this?"
    elif q == 3:
        what = "How do you achieve your goal?"
    print(what)
    os.system("rosservice call /explainable_bt \"what: '" + what + "'\"")

