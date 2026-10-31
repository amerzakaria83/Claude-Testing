# UAE Lottery "Lucky Day" analyser

`lucky_day_predictor.py` reads the results spreadsheet in `data/`, looks for patterns,
checks each one against what pure chance would produce, back-tests several prediction
strategies, and prints tickets for the next draw.

```bash
pip install openpyxl
python lucky_day_predictor.py                      # full report + 5 tickets
python lucky_day_predictor.py --tickets 10 --strategy hot --seed 42
python lucky_day_predictor.py --only-predict       # just the tickets
```

Strategies: `random`, `hot` (most drawn), `cold` (least drawn), `overdue` (longest unseen),
`weighted` (recent draws count more), `balanced` (random numbers with a typical sum and
odd/even mix; this is the default).

## What the data shows (57 draws, Feb 2025 – Oct 2026)

| Pattern checked | What we saw | Verdict |
|---|---|---|
| Day number frequency | 20 drawn most (18x), 6 drawn least (6x); about 11x expected | Normal for random draws (p = 0.75) |
| Lucky Month | Month 1 drawn 10x, about 4.8x expected | Normal once you allow for checking all 12 months (p = 0.22) |
| Sum of the 6 days | Average 99, random gives 96 | Normal |
| Odd/even split | Close to the random distribution | Normal |
| Repeats from previous draw | 1.04 on average, random gives 1.16 | Normal |
| "Popular" pairs | 20-25 drawn together 7 times | Normal: with 465 possible pairs, some always look popular |

**Back-test:** each strategy predicted every past draw using only the draws before it.
All of them matched about 1.16 day numbers per ticket, the same as random picks.
The month was right about 8% of the time (1 in 12). None of the strategies did better than chance.

## The honest bottom line

The draws look like a fair random process. A fair draw has no memory, so every ticket has
the same 1 in 8,835,372 chance of matching all 6 days plus the month, whichever strategy
picked it. The script can generate tickets, but it cannot make them more likely to win.

Data note: the spreadsheet is compiled from news reports and unofficial trackers and has gaps.
Check results against theuaelottery.ae.
