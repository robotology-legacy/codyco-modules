struct Gains {
    1: double lambdaProportional;
    2: double lambdaIntegral;
}

struct Reference {
    1: double baseline;
    2: double amplitude;
    3: double frequency;
    4: double phase;
}

struct Settings {
    1: bool enableCommands;
    2: Gains gains;
    3: Reference reference;
}
