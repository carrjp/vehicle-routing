Steps to run vehicle_routing.py:
    Install requirements in the environment that will be running vehicle_routing.py.
        pip install -r requirements.txt

    Run vehicle_routing.py against a single problem.
        python vehicle_routing.py your_test_problem.txt

    Run vehicle_routing.py against a directory of problems.
        python evaluateShared.py --cmd "python vehicle_routing.py" --problemDir training_problems

References:
    https://developers.google.com/optimization/routing