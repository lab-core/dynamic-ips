from setuptools import setup, Extension
import pybind11

# Define CPLEX paths
cplex_include_dir = "/Applications/CPLEX_Studio2211/cplex/include"
concert_include_dir = "/Applications/CPLEX_Studio2211/concert/include"
cpoptimizer_include_dir = "/Applications/CPLEX_Studio2211/cpoptimizer/include"
cplex_lib_dir = "/Applications/CPLEX_Studio2211/cplex/lib/arm64_osx/static_pic"
concert_lib_dir = "/Applications/CPLEX_Studio2211/concert/lib/arm64_osx/static_pic"
cpoptimizer_lib_dir = "/Applications/CPLEX_Studio2211/cpoptimizer/lib/arm64_osx/static_pic"
pybind_include_dir = "pybind11/include"

ext_modules = [
    Extension(
        'CG_binding',
        sources=[
            'src/CG_binding.cpp',
            'src/solver/solver.cpp',
            'src/solver/MasterAlgorithm.cpp',
            'src/solver/LabelingSubProblem.cpp',
            'src/solver/SubproModeler.cpp',
            'src/solver/MasterModeler.cpp',
            'src/data/Vehicle.cpp',
            'src/data/Instance.cpp',
            'src/data/Task.cpp',
            'src/data/Graph.cpp',
            'src/data/Route.cpp',
            'src/data/Parameters.cpp',
            'src/data/Stop.cpp',
            'src/data/Station.cpp',
            'src/data/Label.cpp',
            'src/utilities/InputPaths.cpp',
            'src/utilities/MyTools.cpp',
            'src/utilities/ReadWrite.cpp',
            'src/utilities/Tools.cpp',
        ],
        include_dirs=[
            pybind11.get_include(),
            'src',
            'src/solver',
            'src/data',
            'src/utilities',
            cplex_include_dir,
            concert_include_dir,
            cpoptimizer_include_dir,
            pybind_include_dir,
        ],
        library_dirs=[
            cplex_lib_dir,
            concert_lib_dir,
            cpoptimizer_lib_dir,
        ],
        libraries=['cplex', 'ilocplex', 'concert'],
        language='c++',
        extra_compile_args=['-std=c++14', '-stdlib=libc++'],  # Specify the C++ standard
    ),
]

setup(
    name='CG_binding',
    version='0.1.0',
    description='Bike Sharing Optimization Module',
    ext_modules=ext_modules,
    zip_safe=False,
)
