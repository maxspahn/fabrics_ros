import logging
import optuna
import joblib
import csv
import argparse
from optuna.visualization import plot_optimization_history, plot_param_importances
import urdfenvs.panda_reacher 
import numpy as np
import matplotlib.pyplot as plt



from optuna_fabrics.tune.fabrics_trial import FabricsTrial

logging.basicConfig(level=logging.INFO)
optuna.logging.set_verbosity(optuna.logging.INFO)

class FabricsStudy(object):
    def __init__(self, trial: FabricsTrial):
        self.initialize_argument_parser()
        cli_arguments = self._parser.parse_args()
        self._trial = trial
        self._number_trials = cli_arguments.number_trials
        self._input_file=cli_arguments.input
        self._output_file=cli_arguments.output
        self._evaluate = cli_arguments.evaluate
        self._manual_tuning = cli_arguments.manual_tuning
        self._render = cli_arguments.render
        self._shuffle = cli_arguments.shuffle
        if cli_arguments.seed >= 0:
            np.random.seed(cli_arguments.seed)
        if not self._manual_tuning:
            self.initialize_study()
        if not self._output_file:
            self._output_file = "temp_optuna_output.pkl"

    def initialize_argument_parser(self):
        self._parser = argparse.ArgumentParser()
        self._parser.add_argument("--input", "-i", type=str, help="Study to be loaded")
        self._parser.add_argument("--output", "-o", type=str, help="Path to save study")
        self._parser.add_argument("--number_trials", "-n", type=int, default=10)
        self._parser.add_argument("--render", "-r", action='store_true')
        self._parser.add_argument("--evaluate", "-e", action='store_true')
        self._parser.add_argument("--manual_tuning", "-m", action="store_true")
        self._parser.add_argument("--no-shuffle", "-ns", action="store_false", dest="shuffle")
        self._parser.add_argument("--seed", "-s", type=int, default=-1)
        self._parser.set_defaults(render=False, manual_tuning=False, evaluate=False, shuffle=True)

    def tune(self):
        # Let us minimize the objective function above.
        render = False
        (env, obstacles, goal, initial_observation) = self._trial.initialize_environment(render=render, shuffle=self._shuffle)
        q0 = self._trial.q0()
        planner = self._trial.set_planner(goal)
        logging.info(f"Running {self._number_trials} trials...")
        self._study.optimize(
            lambda trial: self._trial.objective(trial, planner, obstacles, goal, env, q0),
            n_trials=self._number_trials,
        )
        #print("Best value: {} (params: {})\n".format(study.best_value, study.best_params))
        self.save_study()

    def save_study(self):
        logging.info(f"Saving study to {self._output_file}")
        joblib.dump(self._study, self._output_file)

    def test_result(self):
        if self._manual_tuning:
            params = self._trial.manual_parameters()
        else:
            params = self._study.best_params
        print(f"Selected parameters: {params}")
        total_costs = []
        for i in range(self._number_trials):
            (env, obstacles, goal, _) = self._trial.initialize_environment(render=self._render, shuffle=self._shuffle)
            q0 = self._trial.q0()
            ob = env.reset(pos=q0)
            planner = self._trial.set_planner(goal)
            for obst in obstacles:
                env.add_obstacle(obst)
            env.add_goal(goal)
            costs = self._trial.run(params, planner, obstacles, ob, goal, env)
            logging.info(f"Finished test run {i} with cost: {costs}")
            total_costs.append(costs)
        with open("result.csv", "w") as f:
            writer = csv.writer(f)
            for cost in total_costs:
                writer.writerow([cost])
        logging.info(f"Finished test run with average costs: {np.mean(total_costs)}")

    def show_history(self):
            #fig = plot_optimization_history(self._study)
            fig = plot_param_importances(self._study)
            fig.update_layout(
                font=dict(
                    family="Serif",
                    size=30,
                    color="black"
                )
            )
            fig.show()

    def initialize_study(self) -> None:
        if self._input_file:
            logging.info(f"Reading study from {self._input_file}")
            self._study = joblib.load(self._input_file)
        else:
            logging.info(f"Creating new study")
            self._study = optuna.create_study(direction="minimize")

    def run(self):
        if not self._evaluate:
            self.tune()
        else:
            self.test_result()


