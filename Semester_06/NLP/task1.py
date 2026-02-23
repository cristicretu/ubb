import spacy
from spacy import displacy

eng = [
    "I think Google just laid off like 200 people from their cloud division.",
    "My friend Sarah moved to London last year and she hates the weather there.",
    "The bus was 20 minutes late again so I missed my morning lecture.",
    "Obama gave this really good speech at the UN back in 2015 or something.",
    "We went to that new Thai place on Market Street and the food was mid honestly.",
    "Tesla stock dropped like 8 percent after Musk tweeted some weird stuff.",
    "I forgot to buy milk at Lidl and now I have to go back tomorrow morning.",
    "The professor cancelled the exam and gave us a project instead which is worse.",
    "Barcelona lost 3-0 to Bayern Munich and everyone at the bar was so mad.",
    "Netflix raised their prices again and I'm seriously considering cancelling.",
]

ro = [
    "Am uitat sa iau paine de la Kaufland si acum tre sa ma duc iar.",
    "Ieri a plouat toata ziua si n-am putut sa ies din casa deloc.",
    "Baietii de la Universitatea Cluj au castigat meciul cu Dinamo pana la urma.",
    "Trenul de Bucuresti a avut intarziere doua ore ca de obicei.",
    "Am vazut un film pe HBO Max aseara dar nu mai tin minte cum se numea.",
    "La Cluj s-a deschis un restaurant nou langa Piata Unirii si e destul de bun.",
    "Colegul meu de camera a mancat tot ce era in frigider fara sa intrebe.",
    "Profesorul de la mate ne-a dat tema de trei ori mai multa decat saptamana trecuta.",
    "Am ramas fara baterie la telefon fix cand aveam nevoie de GPS.",
    "Preturile la benzina iar au crescut si nu mai am bani de nimic.",
]

nlp_en = spacy.load("en_core_web_sm")
nlp_ro = spacy.load("ro_core_news_sm")

f = open("task1_results.txt", "w")

def out(text):
    print(text)
    f.write(text + "\n")

def process(sentences, nlp, has_chunks=True):
    for i, sent in enumerate(sentences):
        doc = nlp(sent)
        out(f"\n[{i+1}] {sent}")
        out(f"  tokens: {[t.text for t in doc]}")
        out(f"  lemmas: {[t.lemma_ for t in doc]}")
        out(f"  pos: {[(t.text, t.pos_) for t in doc]}")
        out(f"  deps: {[(t.text, t.dep_, t.head.text) for t in doc]}")
        if has_chunks:
            out(f"  chunks: {[(c.text, c.root.dep_) for c in doc.noun_chunks]}")
        out(f"  ner: {[(e.text, e.label_) for e in doc.ents]}")
        out(f"  dep tree:")
        for t in doc:
            out(f"    {t.text} --{t.dep_}--> {t.head.text}")

out("== English ==")
process(eng, nlp_en)

out("\n== Romanian ==")
process(ro, nlp_ro, has_chunks=False)

f.close()

all_en = list(nlp_en.pipe(eng))
all_ro = list(nlp_ro.pipe(ro))

displacy.serve(all_en + all_ro, style="dep", port=5432)
