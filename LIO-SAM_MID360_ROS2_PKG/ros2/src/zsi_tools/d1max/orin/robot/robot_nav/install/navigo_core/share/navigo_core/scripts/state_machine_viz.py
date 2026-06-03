#!/usr/bin/env python3
# -*- coding: UTF-8 -*-
import os
import sys
import traceback

from transitions.extensions import GraphMachine as Machine


def viz_state_machine(out_put_folder, state_machine_name, count_states, count_trans):
    print(f"Starting visualization with {len(count_states)} states and {len(count_trans)} transitions")

    try:
        # Check if we have any states
        if not count_states:
            print("Error: No states found, cannot create state machine")
            return False

        print(f"States: {count_states}")
        print(f"Transitions: {count_trans}")

        # Create state machine with basic configuration
        print("Creating state machine...")
        machine = Machine(
            states=count_states,
            transitions=count_trans,
            initial=count_states[0],
            auto_transitions=False,
            ignore_invalid_triggers=True
        )

        print("Getting graph object...")
        graph = machine.get_graph()

        # Generate PNG output
        output_path = os.path.join(out_put_folder, f"{state_machine_name}.png")
        print(f"Generating output to: {output_path}")

        graph.draw(output_path, prog='dot', args='-Gdpi=300')
        print(f"Successfully generated: {output_path}")
        return True

    except Exception as e:
        print(f"Error in visualization: {e}")
        print("Full traceback:")
        traceback.print_exc()
        return False


def load_states(f):
    print("=== Loading States ===")
    states = []

    try:
        line = f.readline().strip()
        print(f"First line: '{line}'")
        if "begin_state" != line:
            print(f"Error: Expected 'begin_state', got '{line}'")
            return states

        num_line_str = f.readline().strip()
        print(f"Number line: '{num_line_str}'")
        num_line = int(num_line_str)
        print(f"Reading {num_line} state entries...")

        for i in range(num_line):
            line = f.readline().strip()
            print(f"State {i + 1}: '{line}'")
            if line and not line.startswith('T'):
                states.append(line)
                print(f"  -> Added as state: {line}")
            elif line.startswith('T'):
                print(f"  -> Skipped T-prefixed: {line}")

        end_line = f.readline().strip()
        print(f"End line: '{end_line}'")

    except Exception as e:
        print(f"Error loading states: {e}")
        traceback.print_exc()

    print(f"Final states: {states}")
    return states


def load_change_flag(f):
    print("=== Loading Change Flags ===")
    change_flags = []

    try:
        line = f.readline().strip()
        print(f"First line: '{line}'")
        if "begin_change_flag" != line:
            print(f"Error: Expected 'begin_change_flag', got '{line}'")
            return change_flags

        num_line_str = f.readline().strip()
        print(f"Number line: '{num_line_str}'")
        num_line = int(num_line_str)
        print(f"Reading {num_line} change flag entries...")

        for i in range(num_line):
            line = f.readline().strip()
            print(f"Change flag {i + 1}: '{line}'")
            if line:
                change_flags.append(line)
                print(f"  -> Added: {line}")

        end_line = f.readline().strip()
        print(f"End line: '{end_line}'")

    except Exception as e:
        print(f"Error loading change flags: {e}")
        traceback.print_exc()

    print(f"Final change flags: {change_flags}")
    return change_flags


def load_transitions(f):
    print("=== Loading Transitions ===")
    trans = []

    try:
        line = f.readline().strip()
        print(f"First line: '{line}'")
        if "begin_transition" != line:
            print(f"Error: Expected 'begin_transition', got '{line}'")
            return trans

        num_line_str = f.readline().strip()
        print(f"Number line: '{num_line_str}'")
        num_line = int(num_line_str)
        print(f"Reading {num_line} transition entries...")

        for i in range(num_line):
            line = f.readline().strip()
            print(f"Transition {i + 1}: '{line}'")

            if not line:
                print(f"  -> Skipped empty line")
                continue

            parts = line.split()
            print(f"  -> Split into {len(parts)} parts: {parts}")

            if len(parts) >= 3:
                source = parts[0]
                trigger = parts[1]
                dest = parts[2]

                transition = {
                    'trigger': trigger,
                    'source': source,
                    'dest': dest
                }
                trans.append(transition)
                print(f"  -> Added: {source} --[{trigger}]--> {dest}")
            else:
                print(f"  -> Invalid format (need 3 parts): {line}")

        end_line = f.readline().strip()
        print(f"End line: '{end_line}'")

    except Exception as e:
        print(f"Error loading transitions: {e}")
        traceback.print_exc()

    print(f"Final transitions: {len(trans)} items")
    return trans


def load_state_machine(input_folder, state_machine_name, out_put_folder):
    print(f"=== Loading State Machine ===")
    print(f"Input folder: {input_folder}")
    print(f"State machine name: {state_machine_name}")
    print(f"Output folder: {out_put_folder}")

    # Ensure output folder exists
    os.makedirs(out_put_folder, exist_ok=True)

    # Read state machine file
    file_name = input_folder + state_machine_name
    print(f"Full file path: {file_name}")

    if not os.path.exists(file_name):
        print(f"Error: File does not exist: {file_name}")
        return

    try:
        print("Opening file...")
        with open(file_name, mode='r', encoding='utf-8') as f:
            print("File opened successfully")

            states = load_states(f)
            change_flags = load_change_flag(f)
            trans = load_transitions(f)

        print("File reading completed")

    except Exception as e:
        print(f"Error reading file: {e}")
        traceback.print_exc()
        return

    print(f"=== Summary ===")
    print(f"States found: {len(states)}")
    print(f"Change flags found: {len(change_flags)}")
    print(f"Transitions found: {len(trans)}")

    # Validate data integrity
    if not states:
        print("Error: No valid states found! Cannot create state machine.")
        return

    # Filter valid transitions
    all_states_set = set(states)
    valid_transitions = []

    for trans_dict in trans:
        source = trans_dict['source']
        dest = trans_dict['dest']
        trigger = trans_dict['trigger']

        if source in all_states_set and dest in all_states_set:
            valid_transitions.append(trans_dict)
        else:
            print(f"Skipping invalid transition: {source} --[{trigger}]--> {dest}")
            if source not in all_states_set:
                print(f"  Unknown source: {source}")
            if dest not in all_states_set:
                print(f"  Unknown dest: {dest}")

    print(f"Valid transitions: {len(valid_transitions)}")

    # Try visualization
    success = viz_state_machine(out_put_folder, state_machine_name, states, valid_transitions)

    if success:
        print("State machine visualization completed successfully!")
    else:
        print("State machine visualization failed!")


if __name__ == "__main__":
    print("=== State Machine Visualizer ===")

    if len(sys.argv) < 4:
        print("Usage: state_machine_viz.py input_folder state_machine_file_name output_folder")
        print("Example: python state_machine_viz.py ./input/ state_machine.txt ./output/")
        sys.exit(1)

    print(f"Arguments: {sys.argv}")

    try:
        load_state_machine(sys.argv[1], sys.argv[2], sys.argv[3])
    except Exception as e:
        print(f"Fatal error: {e}")
        traceback.print_exc()
