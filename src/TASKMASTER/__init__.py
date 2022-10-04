if __package__ is None or __package__ == '':
    from task_master import TaskMaster, Agent, Action
else:
    from .task_master import TaskMaster, Agent, Action
