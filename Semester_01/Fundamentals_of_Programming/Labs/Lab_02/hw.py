"""
Exchange Sort
Heap Sort

@author: Cretu Cristian, 913
"""
import random


def heapifyList(vectorToHeapify: list, vectorSize: int, startingNode: int) -> None:
    """
    Heapify a subtree rooted at node starting node. This function is used in the Heap Sort algorithm.
    """
    largestElement = startingNode # initialize largest element as root
    leftSide = 2 * startingNode + 1 # left child index
    rightSide = 2 * startingNode + 2 # right child index

    # check if left child of root exists and is greater than root
    if leftSide < vectorSize and vectorToHeapify[startingNode] < vectorToHeapify[leftSide]:
        largestElement = leftSide

    # check if right child of root exists and is greater than root
    if rightSide < vectorSize and vectorToHeapify[largestElement] < vectorToHeapify[rightSide]:
        largestElement = rightSide

    # change root, if needed
    if largestElement != startingNode:
        vectorToHeapify[startingNode], vectorToHeapify[largestElement] = vectorToHeapify[largestElement], vectorToHeapify[startingNode]
        heapifyList(vectorToHeapify, vectorSize, largestElement)


def sortListUsingHeapSortAndUseAStepper(vectorToSort: list, stepToPrintBetweenReadFromTheKeyboard: int) -> list:
    """
    Heap Sort first transforms the unsorted list into a max heap, a specialized tree-based data structure.
    It then repeatedly removes the maximum element from the heap and reconstructs the heap until it is empty,
    resulting in a sorted list.

    :vectorToSort: The vector to sort.
    :stepToPrintBetweenReadFromTheKeyboard: The number of operations after which the list should be displayed.

    :return: The sorted vector.
    """

    totalStepCounter = 0
    stepCounterAfterEachSwap = 0
    vectorSize = len(vectorToSort)

    # build a max heap from the list
    for index in range(vectorSize // 2 - 1, -1, -1):
        heapifyList(vectorToSort, vectorSize, index)

    # extract elements from a heap, one by one
    for index in range(vectorSize - 1, 0, -1):
        # swap the root with the last element
        vectorToSort[index], vectorToSort[0] = vectorToSort[0], vectorToSort[index]

        # heapify the reduced heap
        heapifyList(vectorToSort, index, 0)

        totalStepCounter += 1
        stepCounterAfterEachSwap += 1
        if stepToPrintBetweenReadFromTheKeyboard == stepCounterAfterEachSwap:
            printListToTheConsole(vectorToSort, totalStepCounter)
            stepCounterAfterEachSwap = 0

    return vectorToSort


def sortListUsingExchangeSortAndUseAStepper(vectorToSort: list, stepToPrintBetweenReadFromTheKeyboard: int) -> list:
    """
    Exchange Sort compares each pair of adjacent items and swaps them if they are in the wrong order.
    This process is repeated until the list is sorted. It works by repeatedly "exchanging" misplaced
    elements to move them to their correct positions.

    :vectorToSort: The vector to sort.
    :stepToPrintBetweenReadFromTheKeyboard: The number of operations after which the list should be displayed.

    :return: The sorted vector.
    """

    stepCounterAfterEachSwap = 0
    totalStepCounter = 0

    for i in range(0, len(vectorToSort) - 1):
        for j in range(i + 1, len(vectorToSort)):
            if vectorToSort[i] > vectorToSort[j]:
                vectorToSort[i], vectorToSort[j] = vectorToSort[j], vectorToSort[i]
                stepCounterAfterEachSwap += 1
                totalStepCounter += 1
                if stepToPrintBetweenReadFromTheKeyboard == stepCounterAfterEachSwap:
                    printListToTheConsole(vectorToSort, totalStepCounter)
                    stepCounterAfterEachSwap = 0

    return vectorToSort


def generateListWithRandomNumbers(listLength: int, interval: tuple = (0, 100)) -> list:
    """
    Generates a list of random integers ranging from 0 to 100.

    :listLength: The length of the list to generate.
    :return: The generated list.
    """
    lowerBound, upperBound = interval
    return [random.randint(lowerBound, upperBound) for _ in range(listLength)]


def readInputFromTheKeyboard(messageToDisplay: str) -> int:
    """
    Reads an integer from the keyboard. If the input is not an integer, it will display an error message,
    and it will ask the user to enter a valid integer.
    """
    while True:
        try:
            return int(input(messageToDisplay))
        except ValueError:
            print("[ERROR]: Please enter a valid integer.")


def printListToTheConsole(vectorToPrint: list, step: int | None = None) -> None:
    """
    Prints the list to the console. If a step is provided, it will be displayed in the console.
    """
    if step is None:
        print(vectorToPrint)
        return

    print(f"\nStep {step}: {vectorToPrint}\n")


def printOptionsAvailableToTheUserThroughTheConsole() -> None:
    """
    Prints the options available to the user through the console.
    """
    print("\n-----------------------------------\n")
    print("1. Read input from the console")
    print("2. Generate a list with random numbers")
    print("3. Print list to the console")
    print("4. Sort list using exchange sort and use a stepper")
    print("5. Sort list using heap sort and use a stepper")
    print("0. Exit")
    print("\n-----------------------------------\n")


def mainProgram() -> None:
    """
    The main program where the user can choose what to do with the functions defined above.
    """
    lengthOfTheListReadFromTheKeyboard = 0
    randomlyGeneratedList = []

    while True:
        printOptionsAvailableToTheUserThroughTheConsole()

        optionChosenByTheUser = readInputFromTheKeyboard("Please enter your option: ")
        print("\n-----------------------------------\n")

        if optionChosenByTheUser == 1:
            lengthOfTheListReadFromTheKeyboard = readInputFromTheKeyboard("Please enter the length of the list: ")
        elif optionChosenByTheUser == 2:

            if lengthOfTheListReadFromTheKeyboard == 0:
                print("[ERROR]: Please read the length of the list first.")
                continue
            randomlyGeneratedList = generateListWithRandomNumbers(lengthOfTheListReadFromTheKeyboard)

            print("[OK]: List generated successfully. Use option 3 to print the list.")
        elif optionChosenByTheUser == 3:

            if len(randomlyGeneratedList) == 0:
                print("[ERROR]: Please generate a list first.")
                continue

            printListToTheConsole(randomlyGeneratedList)
        elif optionChosenByTheUser == 4:

            if len(randomlyGeneratedList) == 0:
                print("[ERROR]: Please generate a list first.")
                continue

            stepToPrintBetweenReadFromTheKeyboard = readInputFromTheKeyboard("Please enter the number of operations after which the list should be displayed: ")

            sortedList = sortListUsingExchangeSortAndUseAStepper(randomlyGeneratedList, stepToPrintBetweenReadFromTheKeyboard)
        elif optionChosenByTheUser == 5:

            if len(randomlyGeneratedList) == 0:
                print("[ERROR]: Please generate a list first.")
                continue

            stepToPrintBetweenReadFromTheKeyboard = readInputFromTheKeyboard("Please enter the number of operations after which the list should be displayed: ")

            sortedList = sortListUsingHeapSortAndUseAStepper(randomlyGeneratedList, stepToPrintBetweenReadFromTheKeyboard)
        else:
            print("Exiting...")
            break


if __name__ == "__main__":
    print("(?): This is a list sorting program using exchange sort / heap sort and a stepper. Follow the instructions on screen.")

    mainProgram()

