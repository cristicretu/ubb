import numpy as np
from gensim.models import KeyedVectors

MODELS = {
    "word":  ("corola.300.20.vec",  False),
    "lemma": ("corola.300.50.vec",  False),
    "pos":   ("corola.300.50.pos.vec", True),
}

words = ["democrație", "algoritm", "nostalgie", "răsărit", "efemer", "curaj", "zăpadă", "filosofie", "mântuire", "strălucire"]

analogies = [
    ("rege", "bărbat", "femeie"),       
    ("câine", "lup", "pisică"),          
    ("soare", "zi", "lună"),            
    ("aur", "argint", "fier"),           
]

POS_PREFIX = "Nc"

out = open("task2_results.txt", "w", encoding="utf-8")

def pr(text=""):
    print(text)
    out.write(text + "\n")


def cosine_similarity(a, b):
    return np.dot(a, b) / (np.linalg.norm(a) * np.linalg.norm(b))


def most_similar_manual(model, vec, topn=10, exclude=None):
    exclude = exclude or set()
    scores = []
    for w in model.index_to_key:
        if w in exclude:
            continue
        scores.append((w, cosine_similarity(vec, model[w])))
    scores.sort(key=lambda x: x[1], reverse=True)
    return scores[:topn]


def format_pos(tag):
    """Nc_rege -> rege (Nc)"""
    if "_" in tag:
        pos, word = tag.split("_", 1)
        return f"{word} ({pos})"
    return tag


for name, (path, is_pos) in MODELS.items():
    pr(f"{'=' * 60}")
    pr(f"MODEL: {name}  ({path})")
    pr(f"{'=' * 60}")

    model = KeyedVectors.load_word2vec_format(path, binary=False)
    fmt = format_pos if is_pos else lambda x: x

    for word in words:
        key = f"{POS_PREFIX}_{word}" if is_pos else word
        pr(f"\n'{word}'" + (f"  [{key}]" if is_pos else "") + ":")
        if key not in model:
            pr("  not in vocabulary")
            continue
        for rank, (neighbor, score) in enumerate(most_similar_manual(model, model[key], topn=10, exclude={key}), 1):
            pr(f"  {rank:2d}. {fmt(neighbor):<25s} {score:.4f}")

    pr("\nAnalogies:")
    for a, b, c in analogies:
        keys = [f"{POS_PREFIX}_{w}" if is_pos else w for w in (a, b, c)]
        pr(f"\n{a} - {b} + {c} = ?")
        if any(k not in model for k in keys):
            pr("  missing words")
            continue
        ka, kb, kc = keys
        analogy_vec = model[ka] - model[kb] + model[kc]
        for rank, (w, score) in enumerate(most_similar_manual(model, analogy_vec, topn=5, exclude={ka, kb, kc}), 1):
            pr(f"  {rank}. {fmt(w):<25s} {score:.4f}")

    pr()

out.close()
