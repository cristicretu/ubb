from repository.repository_exception import RepositoryException


class Repository:
    def __init__(self):
        self.__data = {}

    def add(self, element):
        if element.id in self.__data:
            raise RepositoryException(f"Element with id {element.id} already exists!")
        self.__data[element.id] = element

    def remove(self, element_id):
        if element_id not in self.__data:
            raise RepositoryException(f"Element with id {element_id} does not exist!")
        del self.__data[element_id]

    def update(self, element):
        if element.id not in self.__data:
            raise RepositoryException(f"Element with id {element.id} does not exist!")
        self.__data[element.id] = element

    def get_all(self):
        return list(self.__data.values())

    def search(self, field_to_search_for):
        """
        Searches the data for the given field_to_search_for.

        :param field_to_search_for: The field to search for.

        :return: A list of elements that partially (or fully) match the given field_to_search_for (case insensitive).
        """
        field_to_search_for = field_to_search_for.lower()
        matching_elements = []

        for element in self.__data.values():
            for attribute in element.__dict__.values():
                if field_to_search_for in str(attribute).lower():
                    matching_elements.append(element)

        return matching_elements
