class Singleton(type):
    # A dictionary that serves as a registry of singletons
    _registry = {}

    def __call__(cls, *args, **kwargs):
        # If class previously instantiated and added to the registry then
        # return that same object; otherwise, create a new instance and
        # add it to the registry
        if cls not in cls._registry:
            cls._registry[cls] = super(Singleton, cls).__call__(*args, **kwargs)

        return cls._registry[cls]

    @classmethod
    def _remove(cls, target_cls):
        if cls._registry.has_key(target_cls):
            cls._registry.pop(target_cls)

    @classmethod
    def _clear(cls):
        cls._registry.clear()
