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

pr("=" * 70)
pr("Task 1: RoWordNet")
pr("=" * 70)

for pos, word_list in words.items():
    pr(f"\n== {pos.upper()} ==")

    for word in word_list:
        synset_ids = wn.synsets(literal=word)
        pr(f"\n'{word}' - {len(synset_ids)} synsets")

        for i, sid in enumerate(synset_ids, 1):
            s = wn.synset(sid)
            pr(f"\n  [{i}] {sid}")
            pr(f"  Definition: {s.definition}")
            pr(f"  Literals: {s.literals}")
            pr(f"  Literal senses: {s.literals_senses}")
            pno = s.sentiwn
            pr(f"  Sentiment (P/N/O): {pno[0]}, {pno[1]}, {pno[2]}")

            # relations
            for target_id, rel in wn.outbound_relations(sid):
                t = wn.synset(target_id)
                pr(f"    {rel} -> {t.literals}: {t.definition}")

            for source_id, rel in wn.inbound_relations(sid):
                t = wn.synset(source_id)
                pr(f"    {rel} <- {t.literals}: {t.definition}")

            # hypernym chain
            try:
                chain = wn.synset_to_hypernym_root(sid)
                if chain:
                    pr("  Hypernym path:")
                    for j, nid in enumerate(chain):
                        n = wn.synset(nid)
                        pr(f"    {'  ' * j}-> {n.literals}")
            except Exception:
                pass

out.close()
print("\nSaved to task1_results.txt")
