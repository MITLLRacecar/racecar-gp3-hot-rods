# Team Hot Rods
### BWSI 2021 RACECAR Grand Prix Code

Contributors : 
 - [Daniel Gorbunov](https://github.com/dgorbunov)
 - [Daniel Chuang](https://github.com/daniel-chuang)
 - [Joseph Lee](https://github.com/jjosephlee)
 - [Anna Chatterji](https://github.com/anna-chatterji)

All of us pushed an all nighter for this. Thanks for a great month everyone ♥️ 

## Watch

You can watch a video of our record playthrough [here](https://www.youtube.com/watch?v=VQFt_Yp8H8M&ab_channel=DanielChuang). 

## Usage

- `grand_prix_fast.py` runs our record time code for the GP (opting to use line following for certain segments as it is faster).

- `grand_prix.py` runs our code with each module segments as intended, but produces a slower time.

Run `racecar sim grand_prix.py` with the [Simulator Environment](https://github.com/MITLLRacecar/Simulation) open to run the code. 

## Structure

Each Python module inside `labs/grand_prix` contains the code for a particular segment of the track. 

`grand_prix.py` and `grand_prix_fast.py` pass the Racecar object to each of these modules as they detect AR Markers along the course. The file mappings are stored inside a `dict` mapped to an `IntEnum` for AR detection in both Grand Prix files.
