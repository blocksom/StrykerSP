/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */
package hps;

import java.util.logging.Level;
import java.util.logging.Logger;
import javafx.application.Application;
import javafx.application.Platform;
import javafx.beans.property.DoubleProperty;
import javafx.beans.property.IntegerProperty;
import javafx.beans.property.SimpleDoubleProperty;
import javafx.beans.property.SimpleIntegerProperty;
import javafx.beans.value.ChangeListener;
import javafx.beans.value.ObservableValue;
import javafx.collections.FXCollections;
import javafx.collections.ObservableList;
import javafx.event.ActionEvent;
import javafx.event.EventHandler;
import javafx.scene.Group;
import javafx.scene.ParallelCamera;
import javafx.scene.PerspectiveCamera;
import javafx.scene.Scene;
import javafx.scene.SceneAntialiasing;
import javafx.scene.SubScene;
import javafx.scene.control.Button;
import javafx.scene.control.ComboBox;
import javafx.scene.control.Label;
import javafx.scene.layout.StackPane;
import javafx.scene.layout.VBox;
import javafx.scene.paint.Color;
import javafx.scene.paint.PhongMaterial;
import javafx.scene.shape.Box;
import javafx.scene.shape.Sphere;
import javafx.scene.transform.Rotate;
import javafx.scene.transform.Translate;
import javafx.stage.Stage;
import jssc.SerialPort;
import static jssc.SerialPort.MASK_RXCHAR;
import jssc.SerialPortEvent;
import jssc.SerialPortException;
import jssc.SerialPortList;

/**
 *
 * @author Tristan
 */
public class Hps extends Application {

    private final IntegerProperty kneeAngle = new SimpleIntegerProperty();
    private final IntegerProperty hipFlexAngle = new SimpleIntegerProperty();
    private final IntegerProperty hipAddAbAngle = new SimpleIntegerProperty();
    private final IntegerProperty intRotAngle = new SimpleIntegerProperty();
    private final DoubleProperty xCoordKnee = new SimpleDoubleProperty();
    private final DoubleProperty yCoordKnee = new SimpleDoubleProperty();
    private final DoubleProperty zCoordHip = new SimpleDoubleProperty();
    private final DoubleProperty xCoordHip = new SimpleDoubleProperty();
    private final DoubleProperty xCoordCircle = new SimpleDoubleProperty();
    private final DoubleProperty yCoordCircle = new SimpleDoubleProperty();
    private final DoubleProperty x2CoordCircle = new SimpleDoubleProperty();
    private final DoubleProperty zCoordCircle = new SimpleDoubleProperty();

    SerialPort arduinoPort = null;
    ObservableList<String> portList;

    StringBuilder angle1 = new StringBuilder();
    StringBuilder angle2 = new StringBuilder();
    StringBuilder angle3 = new StringBuilder();
    StringBuilder angle4 = new StringBuilder();

    Rotate rxBox = new Rotate(0, 0, 0, 0, Rotate.X_AXIS);
    Rotate ryBox = new Rotate(0, -150, 50, 0, Rotate.Y_AXIS);
    Rotate rzBox = new Rotate(0, -150, 50, 0, Rotate.Z_AXIS);

    Rotate rxBox2 = new Rotate(0, 0, 0, 0, Rotate.X_AXIS);
    Rotate ryBox2 = new Rotate(0, 0, 0, 0, Rotate.Y_AXIS);
    Rotate rzBox2 = new Rotate(0, -150, 0, 0, Rotate.Z_AXIS);

    Rotate circleX = new Rotate(0, 0, 0, 0, Rotate.X_AXIS);
    Rotate circleY = new Rotate(0, 0, 0, 0, Rotate.Y_AXIS);
    Rotate circleZ = new Rotate(0, 0, 0, 0, Rotate.Z_AXIS);

    Rotate cameraX = new Rotate(0, 0, 0, 0, Rotate.X_AXIS);
    Rotate cameraY = new Rotate(0, 0, 0, 0, Rotate.Y_AXIS);
    Rotate cameraZ = new Rotate(0, 0, 0, 0, Rotate.Z_AXIS);

    Translate translateKnee = new Translate();
    Translate translateHip = new Translate();

    Translate sphereA = new Translate();
    Translate sphereB = new Translate();

    Box thigh = new Box(300, 100, 100);
    Box shank = new Box(300, 100, 100);
    Box table = new Box(800, 50, 400);
    Box staticLeg = new Box(600, 100, 100);
    Sphere joint = new Sphere(50);

    int count = 0;
    Boolean receivingMessage = false;
    String st;

    final PhongMaterial redMaterial = new PhongMaterial();
    final PhongMaterial tableMaterial = new PhongMaterial();

    Label intExtLabel = new Label("Hip Internal/External Angle: ");
    Label hipAbAdLabel = new Label("Hip Adduction/Abduction Angle: ");
    Label kneeFlexExtenLabel = new Label("Knee Flexion/Extension Angle: ");
    Label hipFlexExtenLabel = new Label("Hip Flex/Extension Angle: ");

    VBox ports = new VBox();

    @Override
    public void start(Stage primaryStage) {
        intExtLabel.setLayoutX(1540);
        intExtLabel.setLayoutY(163);

        hipAbAdLabel.setLayoutX(1540);
        hipAbAdLabel.setLayoutY(202);

        kneeFlexExtenLabel.setLayoutX(1540);
        kneeFlexExtenLabel.setLayoutY(239);

        hipFlexExtenLabel.setLayoutX(1540);
        hipFlexExtenLabel.setLayoutY(274);

        PerspectiveCamera cameraAerialView = new PerspectiveCamera();
        Button aerialViewBtn = new Button();
        aerialViewBtn.setLayoutX(300);
        aerialViewBtn.setLayoutY(800);
        aerialViewBtn.setText("Change to aerial view");

        PerspectiveCamera cameraFootView = new PerspectiveCamera();
        Button footViewBtn = new Button();
        footViewBtn.setLayoutX(600);
        footViewBtn.setLayoutY(800);
        footViewBtn.setText("Change to foot view");

        PerspectiveCamera cameraSideView = new PerspectiveCamera();
        Button sideViewBtn = new Button();
        sideViewBtn.setLayoutX(900);
        sideViewBtn.setLayoutY(800);
        sideViewBtn.setText("Change to side view");

        ParallelCamera cameraIsoView = new ParallelCamera();
        Button isoViewBtn = new Button();
        isoViewBtn.setLayoutX(1200);
        isoViewBtn.setLayoutY(800);
        isoViewBtn.setText("Change to isometric view");

        kneeAngle.set(0); // knee angle property
        hipFlexAngle.set(0); // hip adduction 
        hipAddAbAngle.set(0); // hip flex angle
        intRotAngle.set(0);
        xCoordKnee.set(0); // xcoord to move second box
        yCoordKnee.set(0); // ycoord to move second box
        xCoordHip.set(0);
        zCoordHip.set(0);

        rxBox.angleProperty().bind(intRotAngle);
        circleX.angleProperty().bind(intRotAngle);
        rzBox.angleProperty().bind(hipFlexAngle);
        circleZ.angleProperty().bind(hipFlexAngle);
        ryBox.angleProperty().bind(hipAddAbAngle);
        circleY.angleProperty().bind(hipAddAbAngle);
        rzBox2.angleProperty().bind(kneeAngle);
        translateKnee.xProperty().bind(xCoordKnee);
        translateKnee.yProperty().bind(yCoordKnee);
        translateHip.xProperty().bind(xCoordHip);
        translateHip.zProperty().bind(zCoordHip);
        sphereA.xProperty().bind(xCoordCircle);
        sphereA.yProperty().bind(yCoordCircle);
        sphereB.xProperty().bind(x2CoordCircle);
        sphereB.zProperty().bind(zCoordCircle);

        Group thighContainer = new Group(thigh);
        Group shankContainer = new Group(shank);
        //Group jointContainer = new Group(joint);
        Group tableContainer = new Group(table);
        Group staticLegContainer = new Group(staticLeg);

        thighContainer.setTranslateX(800);
        shankContainer.setTranslateX(1100);
        thighContainer.setTranslateY(500);
        shankContainer.setTranslateY(500);
        //jointContainer.setTranslateX(950);
        //jointContainer.setTranslateY(500);
        tableContainer.setTranslateX(900);
        tableContainer.setTranslateY(575);
        tableContainer.setTranslateZ(100);
        staticLegContainer.setTranslateX(950);
        staticLegContainer.setTranslateY(500);
        staticLegContainer.setTranslateZ(150);

        thigh.getTransforms().addAll(rzBox, ryBox, rxBox);
        shank.getTransforms().addAll(rzBox, rzBox2, translateKnee, ryBox, translateHip, rxBox);
        joint.getTransforms().addAll(rzBox, ryBox, sphereA, sphereB, rxBox);

        redMaterial.setDiffuseColor(Color.RED);
        tableMaterial.setDiffuseColor(Color.DARKGRAY);
        thigh.setMaterial(redMaterial);
        shank.setMaterial(redMaterial);
        table.setMaterial(tableMaterial);
        staticLeg.setMaterial(redMaterial);

        detectPort();

        final ComboBox comboBoxPorts = new ComboBox(portList);
        comboBoxPorts.valueProperty()
                .addListener(new ChangeListener<String>() {

                    @Override
                    public void changed(ObservableValue<? extends String> observable,
                            String oldValue, String newValue) {

                        System.out.println(newValue);
                        disconnectArduino();
                        connectArduino(newValue);
                    }

                });
        ports.getChildren().addAll(comboBoxPorts);

        Group root = new Group();
        Group subroot = new Group();
        Scene scene = new Scene(root, 1920, 1080, true, SceneAntialiasing.BALANCED);
        SubScene subScene = new SubScene(subroot, 1920, 1080, true, SceneAntialiasing.BALANCED);

        root.getChildren().addAll(ports, isoViewBtn, aerialViewBtn, footViewBtn, sideViewBtn, intExtLabel, hipAbAdLabel, kneeFlexExtenLabel, hipFlexExtenLabel);
        subroot.getChildren().addAll(staticLegContainer, thighContainer, shankContainer, tableContainer);
        root.getChildren().add(subScene);

        primaryStage.setTitle("Hello World!");
        primaryStage.setMaximized(true);
        primaryStage.setScene(scene);

        primaryStage.show();

//        //int ext rot
//        intRotAngle.set(0);
//        
//        // hip add ab angle
//        hipAddAbAngle.set(20);
//        shank.getTransforms().removeAll(ryBox2, rzBox2);
//        zCoordHip.set(300 * Math.sin(-hipAddAbAngle.get() * (Math.PI / 180)));
//        xCoordHip.set(300 - (300 * Math.cos(-hipAddAbAngle.get() * (Math.PI / 180))));
//
//        zCoordCircle.set((300 * Math.sin(-hipAddAbAngle.get() * (Math.PI / 180))) / 2);
//        x2CoordCircle.set((300 - (300 * Math.cos(-hipAddAbAngle.get() * (Math.PI / 180)))) / 2);
//        shank.getTransforms().addAll(ryBox2, rzBox2);
//
//        // knee angle
//        kneeAngle.set(23);
//
//        // hip flexion extension angle
//        shank.getTransforms().removeAll(rzBox2, ryBox2);
//        hipFlexAngle.set(-10);
//
//        xCoordKnee.set(300 - (300 * Math.cos(-hipFlexAngle.get() * (Math.PI / 180))));
//        yCoordKnee.set(-300 * Math.sin(-hipFlexAngle.get() * (Math.PI / 180)));
//        xCoordCircle.set((300 - (300 * Math.cos(-hipFlexAngle.get() * (Math.PI / 180)))) / 2);
//        yCoordCircle.set((-300 * Math.sin(-hipFlexAngle.get() * (Math.PI / 180))) / 2);
//        shank.getTransforms().addAll(rzBox2, ryBox2);

        aerialViewBtn.setOnAction(new EventHandler<ActionEvent>() {
            @Override
            public void handle(ActionEvent event) {
                cameraAerialView.getTransforms().removeAll(cameraX, cameraY, cameraZ);
                cameraAerialView.setTranslateY(-200);
                cameraX.setAngle(-80);
                cameraAerialView.setTranslateZ(300);
                cameraAerialView.getTransforms().addAll(cameraX);
                subScene.setCamera(cameraAerialView);
            }
        });

        footViewBtn.setOnAction(new EventHandler<ActionEvent>() {
            @Override
            public void handle(ActionEvent event) {
                //camera.setTranslateX(-400);
                cameraFootView.getTransforms().removeAll(cameraY, cameraX, cameraZ);
                cameraY.setAngle(-90);
                cameraX.setAngle(-10);
                cameraFootView.setTranslateZ(-1000);
                cameraFootView.setTranslateX(800);
                cameraFootView.getTransforms().addAll(cameraY, cameraX);
                subScene.setCamera(cameraFootView);
            }
        });

        sideViewBtn.setOnAction(new EventHandler<ActionEvent>() {
            @Override
            public void handle(ActionEvent event) {
                //camera.setTranslateX(-400);
                cameraSideView.getTransforms().removeAll(cameraY, cameraX, cameraZ);
                cameraX.setAngle(0);
                cameraY.setAngle(0);
                cameraZ.setAngle(0);
                cameraSideView.setTranslateX(0);
                cameraSideView.setTranslateY(0);
                cameraSideView.setTranslateZ(0);
                cameraSideView.getTransforms().addAll(cameraX, cameraY, cameraZ);
                subScene.setCamera(cameraSideView);
            }
        });

        isoViewBtn.setOnAction(new EventHandler<ActionEvent>() {
            @Override
            public void handle(ActionEvent event) {
                cameraIsoView.getTransforms().removeAll(cameraY, cameraX, cameraZ);
                cameraY.setAngle(-65);
                cameraX.setAngle(-30);
                cameraZ.setAngle(-20);
                cameraIsoView.setTranslateX(300);
                //cameraIsoView.setTranslateY(200);
                cameraIsoView.setTranslateZ(-600);
                cameraIsoView.getTransforms().addAll(cameraX, cameraY, cameraZ);
                subScene.setCamera(cameraIsoView);
            }
        });
    }

    // gets the list of open COM ports on the computer. 
    private void detectPort() {

        portList = FXCollections.observableArrayList();

        String[] serialPortNames = SerialPortList.getPortNames();
        for (String name : serialPortNames) {
            System.out.println(name);
            portList.add(name);
        }
    }

    public boolean connectArduino(String port) {

        System.out.println("connectArduino");

        boolean success = false;
        SerialPort serialPort = new SerialPort(port);
        try {
            serialPort.openPort();
            serialPort.setParams(
                    SerialPort.BAUDRATE_9600,
                    SerialPort.DATABITS_8,
                    SerialPort.STOPBITS_1,
                    SerialPort.PARITY_NONE);
            serialPort.setEventsMask(MASK_RXCHAR);
            serialPort.addEventListener((SerialPortEvent serialPortEvent) -> {
                if (serialPortEvent.isRXCHAR() && serialPortEvent.getEventValue() > 0) {
                    try {
                        /* 
                            Reads in every byte sent in the serial port. 
                            If a \n is found, then package that string and 
                            convert it to the int to make to angle. If a comma 
                            is found, up the count and append the to the next 
                            angle. Endless stream until the application is shut 
                            down. 
                         */
                        byte buffer[] = serialPort.readBytes(serialPortEvent.getEventValue());
                        for (byte b : buffer) {

                            if (b == 44) {
                                count++;
                            } else if (b == 10) {

                                count = 0;
                                String angle1Process = angle1.toString();
                                String angle2Process = angle2.toString();
                                String angle3Process = angle3.toString();
                                String angle4Process = angle4.toString();

                                //toProcess.split(" ");
                                String temp1 = new String();
                                temp1 = angle1Process.trim();

                                String temp2 = new String();
                                temp2 = angle2Process.trim();

                                // to be used
                                String temp3 = new String();
                                temp3 = angle3Process.trim();

                                // to be used
                                String temp4 = new String();
                                temp4 = angle4Process.trim();

                                System.out.println("Angle1: " + angle1Process);
                                System.out.println("Angle2: " + angle2Process);
                                System.out.println("Angle3: " + angle3Process);
                                System.out.println("Angle4: " + angle4Process);
                                try {

                                    angle1.setLength(0);
                                    angle2.setLength(0);
                                    angle3.setLength(0);
                                    angle4.setLength(0);

                                    // first angle calculations
                                    Integer x = Integer.parseInt(temp1);
                                    Platform.runLater(() -> {
                                        intExtLabel.setText("Interior/Ext Angle: " + x);
                                    });
                                    intRotAngle.set(x);

                                    // second angle calculations
                                    Integer y = Integer.parseInt(temp2);
                                    Platform.runLater(() -> {
                                        hipAbAdLabel.setText("Hip Adduction.Abduction Angle: " + y);
                                    });
                                    hipAddAbAngle.set(y);
                                    shank.getTransforms().removeAll(ryBox2, rzBox2);
                                    zCoordHip.set(300 * Math.sin(-hipAddAbAngle.get() * (Math.PI / 180)));
                                    xCoordHip.set(300 - (300 * Math.cos(-hipAddAbAngle.get() * (Math.PI / 180))));

                                    zCoordCircle.set((300 * Math.sin(-hipAddAbAngle.get() * (Math.PI / 180))) / 2);
                                    x2CoordCircle.set((300 - (300 * Math.cos(-hipAddAbAngle.get() * (Math.PI / 180)))) / 2);
                                    shank.getTransforms().addAll(ryBox2, rzBox2);

                                    // third angle calculations
                                    Integer z = Integer.parseInt(temp3);
                                    Platform.runLater(() -> {
                                        kneeFlexExtenLabel.setText("Knee Flexion/Extension Angle: " + z);
                                    });
                                    if (z >= 0) {
                                        kneeAngle.set(z);
                                    }

                                    // last angle calculations
                                    Integer a = Integer.parseInt(temp4);
                                    Platform.runLater(() -> {
                                        hipFlexExtenLabel.setText("Hip Flexion/Extension Angle: " + a);
                                    });
                                    shank.getTransforms().removeAll(rzBox2, ryBox2);
                                    hipFlexAngle.set(a);

                                    xCoordKnee.set(300 - (300 * Math.cos(-hipFlexAngle.get() * (Math.PI / 180))));
                                    yCoordKnee.set(-300 * Math.sin(-hipFlexAngle.get() * (Math.PI / 180)));
                                    xCoordCircle.set((300 - (300 * Math.cos(-hipFlexAngle.get() * (Math.PI / 180)))) / 2);
                                    yCoordCircle.set((-300 * Math.sin(-hipFlexAngle.get() * (Math.PI / 180))) / 2);
                                    shank.getTransforms().addAll(rzBox2, ryBox2);
                                } catch (NumberFormatException e) {
                                    e.printStackTrace();
                                }

                            } else {
                                if (count == 0) {
                                    angle1.append((char) b);
                                    //System.out.println(angle1);
                                } else if (count == 1) {
                                    angle2.append((char) b);
                                } else if (count == 2) {
                                    angle3.append((char) b);
                                } else if (count == 3) {
                                    angle4.append((char) b);
                                }
                                //System.out.println("appending");
                                //System.out.println(sb);

                            }
                        }

                        //Update label in ui thread
                    } catch (SerialPortException ex) {
                        Logger.getLogger(Hps.class.getName())
                                .log(Level.SEVERE, null, ex);
                    } catch (NumberFormatException e) {

                    }
                }
            });

            arduinoPort = serialPort;
            success = true;
        } catch (SerialPortException ex) {
            Logger.getLogger(Hps.class.getName())
                    .log(Level.SEVERE, null, ex);
            System.out.println("SerialPortException: " + ex.toString());
        }

        return success;
    }

    // gets the open port and closes it. 
    public void disconnectArduino() {

        System.out.println("disconnectArduino()");
        if (arduinoPort != null) {
            try {
                arduinoPort.removeEventListener();

                if (arduinoPort.isOpened()) {
                    arduinoPort.closePort();
                }

            } catch (SerialPortException ex) {
                Logger.getLogger(Hps.class.getName())
                        .log(Level.SEVERE, null, ex);
            }
        }
    }

    public void stop() throws Exception {
        disconnectArduino();
        //stop();
    }

    /**
     * @param args the command line arguments
     */
    public static void main(String[] args) {
        launch(args);
    }
}
