#pragma once
#include <stdio.h>

#include <cctype>
#include <string>

class DogTypeRepository {
 private:
  std::vector<std::string> breeds;
  std::vector<std::string> names;

 public:
  DogTypeRepository() {
    std::string breeds[] = {"affenpinscher", "african",       "airedale",
                            "akita",         "appenzeller",   "australian",
                            "basenji",       "beagle",        "bluetick",
                            "borzoi",        "bouvier",       "boxer",
                            "brabancon",     "briard",        "buhund",
                            "bulldog",       "bullterrier",   "cattledog",
                            "chihuahua",     "chow",          "clumber",
                            "cockapoo",      "collie",        "coonhound",
                            "corgi",         "cotondetulear", "dachshund",
                            "dalmatian",     "dane",          "deerhound",
                            "dhole",         "dingo",         "doberman",
                            "elkhound",      "entlebucher",   "eskimo",
                            "finnish",       "frise",         "germanshepherd",
                            "greyhound",     "groenendael",   "havanese",
                            "hound",         "husky",         "keeshond",
                            "kelpie",        "komondor",      "kuvasz",
                            "labradoodle",   "labrador",      "leonberg",
                            "lhasa",         "malamute",      "malinois",
                            "maltese",       "mastiff",       "mexicanhairless",
                            "mix",           "mountain",      "newfoundland",
                            "otterhound",    "ovcharka",      "papillon",
                            "pekinese",      "pembroke",      "pinscher",
                            "pitbull",       "pointer",       "pomeranian",
                            "poodle",        "pug",           "puggle",
                            "pyrenees",      "redbone",       "retriever",
                            "ridgeback",     "rottweiler",    "saluki",
                            "samoyed",       "schipperke",    "schnauzer",
                            "segugio",       "setter",        "sharpei",
                            "sheepdog",      "shiba",         "shihtzu",
                            "span",          "spitz",         "springer",
                            "stbernard",     "terrier",       "tervuren",
                            "vizsla",        "waterdog",      "weimaraner",
                            "whippet",       "wolfhound"};

    std::string names[] = {
        "Max",     "Buddy",   "Charlie", "Jack",    "Cooper",  "Rocky",
        "Toby",    "Tucker",  "Jake",    "Bear",    "Duke",    "Teddy",
        "Oliver",  "Riley",   "Bailey",  "Bentley", "Milo",    "Max",
        "Buddy",   "Charlie", "Jack",    "Cooper",  "Rocky",   "Toby",
        "Tucker",  "Jake",    "Bear",    "Duke",    "Teddy",   "Oliver",
        "Riley",   "Bailey",  "Bentley", "Milo",    "Max",     "Buddy",
        "Charlie", "Jack",    "Cooper",  "Rocky",   "Toby",    "Tucker",
        "Jake",    "Bear",    "Duke",    "Teddy",   "Oliver",  "Riley",
        "Bailey",  "Bentley", "Milo",    "Max",     "Buddy",   "Charlie",
        "Jack",    "Cooper",  "Rocky",   "Toby",    "Tucker",  "Jake",
        "Bear",    "Duke",    "Teddy",   "Oliver",  "Riley",   "Bailey",
        "Bentley", "Milo",    "Max",     "Buddy",   "Charlie", "Jack",
        "Cooper",  "Rocky",   "Toby",    "Tucker",  "Jake",    "Bear",
        "Duke",    "Teddy",   "Oliver",  "Riley",   "Bailey",  "Bentley",
        "Milo"};

    for (int i = 0; i < sizeof(breeds) / sizeof(breeds[0]); i++) {
      this->breeds.push_back(breeds[i]);
    }

    for (int i = 0; i < sizeof(names) / sizeof(names[0]); i++) {
      this->names.push_back(names[i]);
    }
  }
  ~DogTypeRepository() {}

  std::vector<std::string> getBreeds() const { return this->breeds; }

  int getNumberOfBreeds() const { return this->breeds.size(); }
  std::string getBreed(int index) const { return this->breeds[index]; }

  int findBreed(const std::string &breed) const {
    /// Check if a breed is in the list of breeds
    for (int i = 0, n = this->breeds.size(); i < n; i++) {
      if (this->breeds[i].size() == breed.size() &&
          std::equal(this->breeds[i].begin(), this->breeds[i].end(),
                     breed.begin(), [](auto a, auto b) {
                       return std::tolower(a) == std::tolower(b);
                     })) {
        return i;
      }
    }

    return -1;
  }

  std::pair<std::string, bool> getDogImageUrl(const std::string &breed) {
    /*
    Use the dog.ceo API to get a random image of a dog of a certain breed, so
    that the user can see what the dog looks like.

    Instead of using a library like libcurl, we will use the system call curl,
    and then parse the output.

    :param breed: The breed of the dog
    :return: A pair containing the URL of the image and a boolean indicating
    whether the API call was successful
    */
    char buffer[128];
    std::string result = "";

    std::string to_lowercase = breed;
    for (int i = 0; i < to_lowercase.size(); i++) {
      to_lowercase[i] = std::tolower(to_lowercase[i]);
    }

    std::string command =
        "curl -s https://dog.ceo/api/breed/" + to_lowercase + "/images/random";

    FILE *pipe = popen(command.c_str(), "r");
    if (!pipe) {
      return std::make_pair("", false);
    }

    while (!feof(pipe)) {
      if (fgets(buffer, 128, pipe) != NULL) {
        result += buffer;
      }
    }

    pclose(pipe);

    // std::cout << result << std::endl;
    // {"message":"https:\/\/images.dog.ceo\/breeds\/labrador\/lab_young.jpg","status":"success"}

    std::string res = removeEscapeSlashes(result);

    std::size_t url_start = res.find("https://");
    if (url_start == std::string::npos) {
      return std::make_pair("", false);
    }

    std::size_t url_end = res.find("\"", url_start);
    // if (url_end == std::string::npos) {
    //   return std::make_pair("", false);
    // }

    std::string url = res.substr(url_start, url_end - url_start);
    std::size_t last_slash = url.find_last_of('/');
    // if (last_slash == std::string::npos) {
    //   return std::make_pair("", false);
    // }

    std::string slug = url.substr(last_slash + 1);
    std::string full_url =
        "https://images.dog.ceo/breeds/" + to_lowercase + "/" + slug;
    return std::make_pair(full_url, true);
  }

  std::string removeEscapeSlashes(const std::string &input) {
    /*
    The raw output of the curl command contains escaped slashes. This function
    removes them, such that we can use the normal URL.
    */
    std::string output;
    for (size_t i = 0; i < input.length(); ++i) {
      if (input[i] == '\\' && i + 1 < input.length() && input[i + 1] == '/') {
        continue;
      }
      output += input[i];
    }
    return output;
  }

  std::string getRandomBreed() const {
    int index = rand() % this->breeds.size();
    return this->breeds[index];
  }

  std::string getRandomName() const {
    int index = rand() % this->names.size();
    return this->names[index];
  }
};
