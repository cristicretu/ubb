# Natural Language Processing

Semester 6 · Year 3 · Python, spaCy, RoWordNet, gensim

Classic NLP tooling applied to English and Romanian text: tokenization, tagging, parsing, lexical databases and word embeddings. Each lab writes its output to a `*_results.txt` file next to the script.

## Contents

| Folder | What it is |
| --- | --- |
| [Labs/Lab_01](Labs/Lab_01) | spaCy on 10 English and 10 Romanian sentences: tokens, lemmas, POS tags, dependencies, noun chunks (English only), named entities and a printed dependency tree |
| [Labs/Lab_02](Labs/Lab_02) | Task 1: RoWordNet lookups for Romanian words (synsets, definitions, relations, hypernym paths, sentiment). Task 2: CoRoLa word embeddings with gensim (nearest neighbors via manual cosine similarity, analogies) on word, lemma and POS-tagged models |

## How to run

Lab 1:

```sh
cd Labs/Lab_01
pip install -r requirements.txt
python -m spacy download en_core_web_sm
python -m spacy download ro_core_news_sm
python task1.py
```

Lab 2:

```sh
cd Labs/Lab_02
pip install rowordnet gensim numpy
python task1_rowordnet.py
python task2_embeddings.py
```

## Notes

- `task2_embeddings.py` needs the CoRoLa vector files `corola.300.20.vec`, `corola.300.50.vec` and `corola.300.50.pos.vec` in `Labs/Lab_02`. They are large and git-ignored, so download them yourself.
- The committed `*_results.txt` files show the expected output if you just want to read the results.
