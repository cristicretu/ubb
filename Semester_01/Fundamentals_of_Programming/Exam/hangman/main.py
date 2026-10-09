sentence = "ana has apples"

word = "hangman"
index = 0

words = "abcdefghijklmnopqrstuvwxyz"

known_words = {x: False for x in words}


for w in sentence.split(" "):
    known_words[w[0]] = True
    known_words[w[-1]] = True

while True:
    revealed_word = [" " if x == " " else x if known_words[x] == True else "_" for x in sentence]

    ok = True
    for x in revealed_word:
        if x == "_":
            ok = False

    if index == len(word) or ok == True:
        break

    print("".join(revealed_word))
    print(word[:index])

    letter = input("Enter a letter: ")
    if letter in sentence:
        known_words[letter] = True
    else:
        index += 1
        print("Wrong letter")


    

    
    