from gensim.models import KeyedVectors

model = KeyedVectors.load_word2vec_format("corola.300.20.vec", binary=False)

words = ["casă", "carte", "școală", "om", "frumos", "mare", "bun", "merge", "scrie", "putere"]

analogies = [
    ("rege", "bărbat", "femeie"),
    ("berlin", "germania", "franța"),
    ("frate", "bărbat", "femeie"),
    ("bucurești", "românia", "franța"),
]

out = open("task2_results.txt", "w", encoding="utf-8")

def pr(text=""):
    print(text)
    out.write(text + "\n")

for word in words:
    pr(f"\n'{word}':")
    if word not in model:
        pr("  not in vocabulary")
        continue
    for rank, (neighbor, score) in enumerate(model.most_similar(word, topn=10), 1):
        pr(f"  {rank:2d}. {neighbor:<20s} {score:.4f}")

pr("\nAnalogies:")
for a, b, c in analogies:
    pr(f"\n{a} - {b} + {c} = ?")
    if any(w not in model for w in (a, b, c)):
        pr("  missing words")
        continue
    for rank, (w, score) in enumerate(model.most_similar(positive=[a, c], negative=[b], topn=5), 1):
        pr(f"  {rank}. {w:<20s} {score:.4f}")

out.close()
