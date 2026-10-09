# Artificial Intelligence

Semester 4 · Year 2 · Python, Jupyter, scikit-learn, PyTorch

Machine learning basics with scikit-learn and PyTorch (regression, decision trees, perceptrons, MLPs, CNNs), then classic AI: search problems and genetic algorithms. Most labs are Jupyter notebooks handed out by the teacher and filled in during the lab.

## Contents

| Folder | What it is |
| --- | --- |
| [Labs/Lab_01](Labs/Lab_01) | Python, NumPy, matplotlib and PIL warm-up exercises |
| [Labs/Lab_02](Labs/Lab_02) | Preprocessing (scaling, encoding) and linear regression with Ridge/Lasso/ElasticNet on salary, telco churn and score datasets |
| [Labs/Lab_03](Labs/Lab_03) | Decision tree classifier on a modified Iris dataset, with `GridSearchCV` |
| [Labs/Lab_04](Labs/Lab_04) | PyTorch tensors, autograd, activation functions, a simple perceptron |
| [Labs/Lab_05](Labs/Lab_05) | Multilayer perceptron in PyTorch |
| [Labs/Lab_06](Labs/Lab_06) | Classification models in PyTorch on `make_circles`/`make_blobs`; `utils.py` plots decision boundaries |
| [Labs/Lab_07](Labs/Lab_07) | Computer vision: CNN on FashionMNIST with `torchvision` |
| [Labs/Lab_08](Labs/Lab_08) | Jealous Husbands river crossing puzzle solved with backtracking, DFS and A* |
| [Labs/Lab_09](Labs/Lab_09) | Genetic algorithm (real and binary encoding, several crossovers) on the Ackley and Bukin functions, with plots, statistical comparison and a LaTeX report |

## How to run

```sh
python3 -m venv .venv && source .venv/bin/activate
pip install jupyter numpy pandas matplotlib seaborn scikit-learn torch torchvision pillow tqdm scipy
jupyter notebook
```

Run notebooks from their own folder so the CSV files load. Lab_09 also runs as a script:

```sh
cd Labs/Lab_09
python3 main.py
```

## Notes

- Several notebooks are named `*-unsolved`; that is the teacher's file name. The exercises are mostly solved.
- Lab_07 downloads FashionMNIST into `data/` on first run (ignored by git).
- `Lab_06/models/` holds a saved perceptron (`.pth`) produced by the notebook.
