# New Research Study on Exploratory Testing

**Reference paper:** An experiment on the effectiveness and efficiency of exploratory testing — Afzal, Ghazi, Itkonen, Torkar, Andrews, Bhatti, *Empirical Software Engineering* 20(2):844–878, 2015.
**DOI:** 10.1007/s10664-014-9301-4

**Team members (Group 933):** Cretu Cristian, Grancea Alexandru, Deaconu Victor, Draguta Vasile

**Proposed title:** *Does AI-assisted Exploratory Testing change the effectiveness/efficiency trade-off? A replication of Afzal et al. (2015) with LLM-supported testers in higher education.*

---

## Goal (Goal–Question–Metric)

**Analyse** the practice of exploratory testing
**for the purpose of** evaluating the impact of an LLM-based testing assistant (a chat model with charter context but no test-case generation) on tester productivity
**with respect to** defect-detection efficiency, defect-detection effectiveness (difficulty / type / severity), and false-defect rate
**from the point of view of** the tester and the testing researcher
**in the context of** MSc Software Verification & Validation students performing time-boxed manual functional testing on a medium-size open-source application (jEdit, replicating Afzal et al.'s SUT).

## Research Questions

- **RQ1 (efficiency).** Do testers performing ET with an LLM assistant find more defects in a fixed 90-minute session than testers performing classic ET without an LLM?
- **RQ2 (effectiveness).** Do AI-assisted ET sessions differ from unassisted ET sessions in the *difficulty*, *type* and *severity* of the detected defects?
- **RQ3 (cost of assistance).** Does the LLM assistant increase the number of *false defect reports* compared to unassisted ET?
- **RQ4 (process).** How does the assistant change the testing *flow* — proportion of time spent on charter reading, exploration, oracle reasoning, note-taking?

## Methodology

Controlled experiment with one factor (testing approach) and two treatments — `ET-plain` and `ET-LLM` — replicating Afzal et al.'s design. ~40 top-performing MSc students, randomised into balanced groups, each runs one 90-minute session on jEdit 4.2 seeded with the same 25 faults as in the original paper, preceded by a 15-minute briefing and a common charter. The `ET-LLM` group has access to a chat interface backed by a fixed model (e.g. Claude Sonnet, temperature 0) with the user-manual chapters and the charter pre-loaded into the system prompt; chat logs are recorded. Both groups submit a defect report and a session log; defects are independently triaged for type (IEEE 1044 categories), severity (1–3), and detection difficulty (4 modes). A short post-session survey collects perceived helpfulness, trust, and friction. Statistical analysis follows the reference paper: Mann–Whitney U at α=0.05 for between-group comparisons, Vargha–Delaney Â₁₂ for effect size, and Wilcoxon for repeated-measures sub-analyses.

## Metrics

| Construct | Metric | Collection |
|---|---|---|
| Efficiency | # true defects per 90-min session | Defect-report triage |
| Efficiency (process) | Defects per active testing minute (excluding chat) | Session log + chat log |
| Effectiveness — difficulty | Defects per detection-difficulty mode (1–4) | Triage on Afzal et al.'s scheme |
| Effectiveness — type | Defects per IEEE 1044 type | Triage |
| Effectiveness — severity | Defects per severity level (1–3) | Triage |
| False-defect cost | # invalid defect reports / total reports | Triage |
| AI-interaction load | Chat turns; tokens in/out; % of session in chat | Chat-tool telemetry |
| Tester perception | Likert scores: helpfulness, trust, friction | Post-session survey |
| Confounders | Years of dev/testing experience, prior LLM usage | Pre-session demographic form |
