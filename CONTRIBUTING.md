# Contributing

Thanks for your interest in this SA-LIVO reproduction. This project values
reproducible, evidence-backed changes.

## Reporting issues

- Use the issue templates (bug report / feature request) under
  `.github/ISSUE_TEMPLATE/`.
- For bugs, include: the sequence, the exact configuration (params file and
  overrides), the command used, the estimated trajectory pose count, and the
  official evaluator output.
- For performance claims, provide the reproducible command and the evaluator
  output; numbers are accepted only when produced by a deterministic replay
  and verified with the official evaluators.

## Submitting code

1. Fork the repository and create a feature branch.
2. Keep changes focused; add or update tests in `sa_livo/tests/` where
   relevant (`saif_test` covers the fusion core).
3. Run the existing tests before submitting:

   ```bash
   cd sa_livo/tests && g++ -std=c++17 -O2 -I/usr/include/eigen3 -I../include \
     saif_test.cpp ../src/saif.cpp -o saif_test && ./saif_test
   ```

4. Follow the existing code style and add comments for non-obvious logic.
5. Open a pull request using the PR template; describe what changed, why, and
   how it was verified (including any regression results on the benchmark
   sequences).

## Ground rules

- Do not commit secrets, internal paths, or personal data.
- Do not commit large binaries (logs, PCDs, bags); keep them out of git.
- Performance numbers must be reproducible with a documented command.
