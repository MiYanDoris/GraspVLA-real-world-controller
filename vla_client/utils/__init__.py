def input_typed(prompt: str, tp: type):
    if tp == bool:
        tp_fn = lambda x: {'y': True, 'n': False}[x]
    else:
        tp_fn = tp
    while True:
        try:
            return tp_fn(input(prompt))
        except:
            print('invalid input')


def input_choice(prompt: str, choices: list):
    print('please choose from:')
    for i, choice in enumerate(choices):
        print(f'{i}: {choice}')
    while True:
        try:
            choice = int(input(prompt))
            assert choice in range(len(choices))
            return choices[choice]
        except KeyboardInterrupt:
            raise
        except:
            print('invalid input')
