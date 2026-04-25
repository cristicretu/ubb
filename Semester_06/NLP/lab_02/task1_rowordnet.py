import rowordnet as rwn

wn = rwn.RoWordNet()

words = {
    "nouns": ["casă", "carte", "drum"],
    "verbs": ["merge", "scrie", "gândi"],
    "adjectives": ["frumos", "mare"],
}

out = open("task1_results.txt", "w", encoding="utf-8")

def pr(text=""):
    print(text)
    out.write(text + "\n")

for pos, word_list in words.items():
    pr(f"\n{pos.upper()}")

    for word in word_list:
        synset_ids = wn.synsets(literal=word)
        pr(f"\n'{word}' - {len(synset_ids)} synsets")

        for i, sid in enumerate(synset_ids, 1):
            s = wn.synset(sid)
            pno = s.sentiwn
            pr(f"\n  [{i}] {sid}")
            pr(f"  def: {s.definition}")
            pr(f"  literals: {s.literals}, senses: {s.literals_senses}")
            pr(f"  sentiment: P={pno[0]} N={pno[1]} O={pno[2]}")

            for tid, rel in wn.outbound_relations(sid):
                t = wn.synset(tid)
                pr(f"    {rel} -> {t.literals}")

            try:
                chain = wn.synset_to_hypernym_root(sid)
                if chain:
                    pr("  path: " + " > ".join(str(wn.synset(n).literals) for n in chain))
            except Exception:
                pass

out.close()
