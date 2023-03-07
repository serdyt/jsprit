package com.graphhopper.jsprit.examples;

import com.graphhopper.jsprit.analysis.toolbox.GraphStreamViewer;
import com.graphhopper.jsprit.core.algorithm.VehicleRoutingAlgorithm;
import com.graphhopper.jsprit.core.algorithm.box.GreedySchrimpfFactory;
import com.graphhopper.jsprit.core.algorithm.box.Jsprit;
import com.graphhopper.jsprit.core.algorithm.state.StateId;
import com.graphhopper.jsprit.core.algorithm.state.StateManager;
import com.graphhopper.jsprit.core.algorithm.state.UpdateMaxTimeInVehicle;
import com.graphhopper.jsprit.core.algorithm.termination.IterationWithoutImprovementTermination;
import com.graphhopper.jsprit.core.algorithm.termination.VariationCoefficientTermination;
import com.graphhopper.jsprit.core.problem.Location;
import com.graphhopper.jsprit.core.problem.VehicleRoutingProblem;
import com.graphhopper.jsprit.core.problem.VehicleRoutingProblem.FleetSize;
import com.graphhopper.jsprit.core.problem.constraint.ConstraintManager;
import com.graphhopper.jsprit.core.problem.constraint.MaxTimeInVehicleConstraint;
import com.graphhopper.jsprit.core.problem.job.Break;
import com.graphhopper.jsprit.core.problem.job.Job;
import com.graphhopper.jsprit.core.problem.job.Service;
import com.graphhopper.jsprit.core.problem.job.Shipment;
import com.graphhopper.jsprit.core.problem.solution.VehicleRoutingProblemSolution;
import com.graphhopper.jsprit.core.problem.solution.route.activity.TimeWindow;
import com.graphhopper.jsprit.core.problem.vehicle.VehicleImpl;
import com.graphhopper.jsprit.core.problem.vehicle.VehicleType;
import com.graphhopper.jsprit.core.problem.vehicle.VehicleTypeImpl;
import com.graphhopper.jsprit.core.reporting.SolutionPrinter;
import com.graphhopper.jsprit.core.util.Coordinate;
import com.graphhopper.jsprit.core.util.FastVehicleRoutingTransportCostsMatrix;
import com.graphhopper.jsprit.core.util.Solutions;
import com.graphhopper.jsprit.io.algorithm.AlgorithmConfigXmlReader;
import com.graphhopper.jsprit.io.problem.VrpXMLReader;
import com.graphhopper.jsprit.io.problem.VrpXMLWriter;
import com.graphhopper.jsprit.util.Examples;
import com.graphhopper.jsprit.analysis.toolbox.Plotter;
import com.graphhopper.jsprit.io.algorithm.VehicleRoutingAlgorithms;
import org.apache.commons.lang.ObjectUtils;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;
import java.text.SimpleDateFormat;


import javax.xml.stream.XMLOutputFactory;
import java.util.*;
import java.io.BufferedReader;
import java.io.File;
import java.io.FileReader;
import java.io.IOException;
import java.util.stream.Collectors;

public class DRT_test {

    private final static Logger logger = LoggerFactory.getLogger(VehicleRoutingAlgorithm.class);

    public static void main(String[] args) throws IOException {
        boolean printSolution = false;
        String picFolder = null;
        String vrpFile = null;
        String tdmFile = null;
        String outFile = null;
        String simLog = null;
        String iterations = "2000";
        String threads = "4";
        String construction = "regret_insertion";
        String fast_regret = "false";

        for (int i = 0; i < args.length; i += 2) {
            if (args[i].equals("-printSolution")) {
                printSolution = Boolean.parseBoolean(args[i+1]);
            }
            else if (args[i].equals("-vrpFile")){
                vrpFile = args[i+1];
            }
            else if (args[i].equals("-tdmFile")){
                tdmFile = args[i+1];
            }
            else if (args[i].equals("-outFile")){
                outFile = args[i+1];
            }
            else if (args[i].equals("-simLog")){
                simLog = args[i+1];
            }
            else if (args[i].equals("-picFolder")){
                picFolder = args[i+1];
            }
            else if (args[i].equals("-iterations")){
                iterations = args[i+1];
            }
            else if (args[i].equals("-threads")){
                threads = args[i+1];
            }
            else if (args[i].equals("-construction")){
                construction = args[i+1];
            }
            else if (args[i].equals("-fast_regret")){
                fast_regret = args[i+1];
            }
        }

        if (vrpFile == null) {
            vrpFile = "data/vrp.xml";
        }
        if (tdmFile == null){
            tdmFile = "data/time_distance_matrix.csv";
        }
        if (outFile == null){
            outFile = "data/problem-with-solution.xml";
        }
        if (simLog == null){
            simLog = "output/log";
        }
        if (picFolder == null){
            picFolder = "pictures/";
        }

        Examples.createOutputFolder();

        VehicleRoutingProblem.Builder vrpBuilder = VehicleRoutingProblem.Builder.newInstance();
        vrpBuilder.setFleetSize(FleetSize.FINITE);

//      The input VRP is either in data/vrp or provided in command line
        new VrpXMLReader(vrpBuilder).read(vrpFile);

//      Calculate the size of time-distance matrix from an input vrp file.
//        Set<Integer> ints = vrpBuilder.getLocationMap().keySet().stream()
//            .map(s -> Integer.parseInt(s))
//            .collect(Collectors.toSet());
//        int tdm_size = java.util.Collections.max(ints)+1;
        int tdm_size = readTdmSize(tdmFile)+1;

//      Read time-distance matrix from an input csv.
//        Input csv file location is either in arguments or in data/time-distance-matrix.csv
        FastVehicleRoutingTransportCostsMatrix.Builder matrixBuilder = FastVehicleRoutingTransportCostsMatrix.Builder
            .newInstance(tdm_size, false);
        initMatrix(matrixBuilder, tdm_size);
        readDistances(matrixBuilder, tdmFile);

        vrpBuilder.setRoutingCost(matrixBuilder.build());

        VehicleRoutingProblem vrp = vrpBuilder.build();

        StateManager stateManager = new StateManager(vrp);
        StateId id = stateManager.createStateId("max-time");
        StateId openJobsId = stateManager.createStateId("open-jobs-id");
        stateManager.addStateUpdater(new UpdateMaxTimeInVehicle(stateManager, id, vrp.getTransportCosts(), vrp.getActivityCosts(), openJobsId));

        ConstraintManager constraintManager = new ConstraintManager(vrp,stateManager);
        constraintManager.addConstraint(new MaxTimeInVehicleConstraint(vrp.getTransportCosts(), vrp.getActivityCosts(), id, stateManager, vrp, openJobsId), ConstraintManager.Priority.CRITICAL);

        Jsprit.Builder algorithmBuilder = Jsprit.Builder.newInstance(vrp);
        algorithmBuilder.setProperty(Jsprit.Parameter.MAX_TRANSPORT_COSTS, Double.toString(1.0E12));
        algorithmBuilder.setProperty(Jsprit.Parameter.THREADS.toString(), threads);
        algorithmBuilder.setProperty(Jsprit.Parameter.ITERATIONS.toString(), iterations);
        algorithmBuilder.setProperty(Jsprit.Parameter.VEHICLE_SWITCH.toString(), "true");
        algorithmBuilder.setProperty(Jsprit.Parameter.BREAK_SCHEDULING.toString(), String.valueOf(true));
        algorithmBuilder.setProperty(Jsprit.Parameter.CONSTRUCTION, construction);
        algorithmBuilder.setStateAndConstraintManager(stateManager,constraintManager);
        algorithmBuilder.setProperty(Jsprit.Parameter.FAST_REGRET.toString(), fast_regret);
        Random rand = new Random();
        rand.setSeed(42);
        algorithmBuilder.setRandom(rand);
        VehicleRoutingAlgorithm algorithm = algorithmBuilder.buildAlgorithm();
        VariationCoefficientTermination prematureTermination = new VariationCoefficientTermination(20, 0.1);
        //algorithm.setPrematureAlgorithmTermination(prematureTermination);
        //algorithm.addListener(prematureTermination);

//        algorithm.setPrematureAlgorithmTermination(new IterationWithoutImprovementTermination(20));
//        algorithm.addTerminationCriterion(new IterationWithoutImprovementTermination(20));

        Collection<VehicleRoutingProblemSolution> solutions = algorithm.searchSolutions();

        SolutionPrinter sprint= new SolutionPrinter();
        sprint.setPrintWriter(simLog);

//        sprint.print(Solutions.bestOf(solutions));

        if (printSolution) {
            new Plotter(vrp, Solutions.bestOf(solutions))
                .plotShipments(true)
                .setBoundingBox(13.43769922,55.46296251,14.36869863,55.78847956)
                .setScalingFactor(1)
                .setLabel(Plotter.Label.ID)
                .plot(picFolder + System.currentTimeMillis() + ".png", "DRT");
        }

        List<VehicleRoutingProblemSolution> l = new ArrayList<>();
        l.add(Solutions.bestOf(solutions));
        new VrpXMLWriter(vrp, l).write(outFile);

        sprint.print(vrp, Solutions.bestOf(solutions), SolutionPrinter.Print.VERBOSE);

//        new GraphStreamViewer(vrp, Solutions.bestOf(solutions))
//            .labelWith(GraphStreamViewer.Label.ID).setRenderShipments(true).setRenderDelay(200)
//            .setGraphStreamFrameScalingFactor(3)
//            .display();
    }

    private static int readTdmSize(String tdmFile) throws IOException {
        BufferedReader br = new BufferedReader(new FileReader(tdmFile));
        String lastLine = "";
        String sCurrentLine;

        while ((sCurrentLine = br.readLine()) != null){
            lastLine = sCurrentLine;
        }
        String[] lineTokens = lastLine.split(",");
        return Integer.parseInt(lineTokens[0]);
    }

    /**
     * Read time distance matrix from csv.
     */
    private static void readDistances(FastVehicleRoutingTransportCostsMatrix.Builder matrixBuilder, String tdmFile) throws IOException {
        BufferedReader reader = new BufferedReader(new FileReader(new File(tdmFile)));
        String line;
        while ((line = reader.readLine()) != null) {
            String[] lineTokens = line.split(",");
            matrixBuilder.addTransportTimeAndDistance(
                Integer.parseInt(lineTokens[0]),
                Integer.parseInt(lineTokens[1]),
                Double.parseDouble(lineTokens[2]),
                Double.parseDouble(lineTokens[3])
            );
        }
        reader.close();
    }

    /**
     * Initiates a cost matrix with maxInt values. Viable routs should be rewritten from an input csv.
     * Moving from a point to itself has zero cost.
     */
    private static void initMatrix(FastVehicleRoutingTransportCostsMatrix.Builder matrixBuilder, int tdm_size){
        for (int i = 0; i < tdm_size; i++){
            for (int j = 0; j < tdm_size; j++) {
                if (i == j){
                    matrixBuilder.addTransportTimeAndDistance(i, j, 0, 0);
                }
                else {
                    matrixBuilder.addTransportTimeAndDistance(i, j, Double.MAX_VALUE, Double.MAX_VALUE);
                }
            }
        }
    }
}
