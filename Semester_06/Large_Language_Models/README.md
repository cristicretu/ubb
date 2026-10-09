# Large Language Models

Semester 6 · Year 3 · Python, Jupyter, Hugging Face Transformers, LangChain, Chroma, DeepEval

Hands-on work with LLMs: running and fine-tuning a small model, prompt templates and chains, retrieval augmented generation (RAG), and evaluating outputs with classic metrics and an LLM judge. Each lab is one Jupyter notebook.

## Contents

| Folder | What it is |
| --- | --- |
| [Labs/Lab_01](Labs/Lab_01) | T5-small: text-to-text prompts with task prefixes, then fine-tuning on two small custom datasets (`explain_dataset.csv`, `roast_dataset.csv`) and comparing with the base model |
| [Labs/Lab_02](Labs/Lab_02) | LangChain on a Hugging Face pipeline: `PromptTemplate`, `LLMChain`, `SequentialChain` (router chain left as a stub) |
| [Labs/Lab_03](Labs/Lab_03) | RAG over PDF lecture notes: load, split, embed into a Chroma vector store (`chroma/`), then `RetrievalQA` with a custom prompt |
| [Labs/Lab_04](Labs/Lab_04) | Evaluation: ROUGE scores, then DeepEval `GEval` correctness with a local model as the judge |

## How to run

```sh
pip install jupyter transformers torch sentencepiece datasets pandas \
  langchain langchain-community langchain-huggingface langchain-openai \
  langchain-text-splitters pypdf chromadb deepeval rouge-score evaluate
jupyter notebook Labs/Lab_01/L1.ipynb
```

Labs 3 and 4 expect an OpenAI-compatible local server at `http://localhost:1234/v1` (LM Studio) with a chat model loaded (`qwen/qwen3.5-9b` in the notebooks) and, for Lab 3, the `text-embedding-nomic-embed-text-v1.5` embedding model.

## Notes

- No API keys needed. Lab 3 passes the placeholder key `lm-studio`, which LM Studio ignores.
- Lab 3 loads PDFs from `docs/`, which is not committed. Use your own PDFs, or reuse the committed `chroma/` store: it was built with the nomic embedding model, so you need that same model to query it.
- Lab 2 defaults to `microsoft/phi-4` (a 14B model). Switch `model_name` to `gpt2` or `t5-small` on a laptop.
- Lab 1 saves fine-tuned models to `t5-custom-response/`, `t5-roast/` and `t5-fine-tuned/` (git-ignored).
- `Labs/Lab_04/.deepeval/.deepeval` is DeepEval's local config that points it at the LM Studio model.
