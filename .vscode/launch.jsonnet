{
    "version": "0.2.0",
    "configurations": [
        {
            "name": "sim",
            "request": "launch",
            "target": std.extVar("SIM_DIR"),
            "type": "ros",
            "env": {
                "SIM_DIR": std.extVar("SIM_DIR"),
                "MODELS_DIR": std.extVar("MODELS_DIR"),
                "PYTHONPATH": std.extVar("PYTHONPATH"),
                "AMENT_PREFIX_PATH": std.extVar("AMENT_PREFIX_PATH")
            },
        },
        {
            "name": "Python: Attach using Process Id",
            "type": "python",
            "request": "attach",
            "processId": "${command:pickProcess}",
        },
  ]
}
