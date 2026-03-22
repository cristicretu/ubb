from gensim.models import KeyedVectors

print("Loading embeddings...")
model = KeyedVectors.load_word2vec_format("corola.300.20.vec", binary=False)
print(f"Loaded {len(model)} vectors, dim={model.vector_size}")

words = ["casă", "carte", "școală", "om", "frumos", "mare", "bun", "merge", "scrie", "putere"]

analogies = [
    ("rege", "bărbat", "femeie"),
    ("paris", "franța", "românia"),
    ("bun", "rău", "frumos"),
    ("bucurești", "românia", "franța"),
]

out = open("task2_results.txt", "w", encoding="utf-8")

def pr(text=""):
    print(text)
    out.write(text + "\n")

pr("=" * 70)
pr(f"Task 2: COROLA Embeddings ({len(model)} words, {model.vector_size}d)")
pr("=" * 70)

# nearest neighbors
pr("\n== NEAREST NEIGHBORS ==")
for word in words:
    pr(f"\n'{word}':")
    if word not in model:
        pr("  not in vocabulary")
        continue
    for rank, (neighbor, score) in enumerate(model.most_similar(word, topn=10), 1):
        pr(f"  {rank:2d}. {neighbor:<25s} {score:.4f}")

# analogies
pr("\n== ANALOGIES ==")
for a, b, c in analogies:
    pr(f"\n{a} - {b} + {c} = ?")
    missing = [w for w in (a, b, c) if w not in model]
    if missing:
        pr(f"  missing: {missing}")
        continue
    for rank, (w, score) in enumerate(model.most_similar(positive=[a, c], negative=[b], topn=5), 1):
        pr(f"  {rank}. {w:<25s} {score:.4f}")

out.close()
print("\nSaved to task2_results.txt")
