import java.util.Arrays;
import java.util.List;
import java.util.stream.Collectors;

public class Main {
  public static void main(String[] args) {
    List<String> words = Arrays.asList("hello", "world", "java", "programming");
    words.forEach(word -> System.out.println("  " + word));

    words.forEach(System.out::println);

    List<String> p31 = words.stream()
        .map(word -> word + "!")
        .collect(Collectors.toList());

    p31.forEach(System.out::println);

    List<String> p32 = words.stream()
        .map(word -> word.replace("i", "eye"))
        .collect(Collectors.toList());

    p32.forEach(System.out::println);

    List<String> p33 = words.stream()
        .map(String::toUpperCase)
        .collect(Collectors.toList());

    p33.forEach(System.out::println);

    List<String> p41 = words.stream()
        .filter(word -> word.length() < 5)
        .collect(Collectors.toList());

    p41.forEach(System.out::println);

    List<String> p42 = words.stream()
        .filter(word -> word.contains("o"))
        .collect(Collectors.toList());

    p42.forEach(System.out::println);

    List<String> p5 = words.stream().filter(word -> word.length() > 5).filter(word -> word.contains("o"))
        .collect(Collectors.toList());

    System.out.println(p5.get(0));

    String p6 = words.stream().reduce((word1, word2) -> word1.toUpperCase() + " " + word2.toUpperCase()).get();
    System.out.println(p6);

    String p7 = words.stream().map(word -> word.toUpperCase()).reduce((word1, word2) -> word1 + " " + word2).get();
    System.out.println(p7);

    int p8 = words.stream().reduce(0, (sum, word) -> sum + word.length(), Integer::sum);
    System.out.println(p8);

    int p9 = words.stream().map(word -> word.contains("o") ? 1 : 0).reduce(0, Integer::sum);
    System.out.println(p9);

  }
}

