/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */
package hps;

import java.io.File;
import java.io.IOException;
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
import javafx.embed.swing.SwingFXUtils;
import javafx.event.ActionEvent;
import javafx.event.EventHandler;
import javafx.fxml.FXMLLoader;
import javafx.scene.Group;
import javafx.scene.ParallelCamera;
import javafx.scene.Parent;
import javafx.scene.PerspectiveCamera;
import javafx.scene.Scene;
import javafx.scene.SceneAntialiasing;
import javafx.scene.SnapshotParameters;
import javafx.scene.SubScene;
import javafx.scene.control.Button;
import javafx.scene.control.ComboBox;
import javafx.scene.control.Label;
import javafx.scene.image.Image;
import javafx.scene.image.ImageView;
import javafx.scene.image.WritableImage;
import javafx.scene.layout.StackPane;
import javafx.scene.layout.VBox;
import javafx.scene.paint.Color;
import javafx.scene.paint.PhongMaterial;
import javafx.scene.shape.Box;
import javafx.scene.shape.Sphere;
import javafx.scene.text.Font;
import javafx.scene.transform.Rotate;
import javafx.scene.transform.Translate;
import javafx.stage.Modality;
import javafx.stage.Stage;
import javax.imageio.ImageIO;
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

   ImageView imv = new ImageView();

   private final IntegerProperty kneeAngle = new SimpleIntegerProperty();
   private final IntegerProperty hipFlexAngle = new SimpleIntegerProperty();
   private final IntegerProperty hipAddAbAngle = new SimpleIntegerProperty();
   private final IntegerProperty intRotAngle = new SimpleIntegerProperty();
   private final DoubleProperty xCoordKnee = new SimpleDoubleProperty();
   private final DoubleProperty yCoordKnee = new SimpleDoubleProperty();
   private final DoubleProperty zCoordHip = new SimpleDoubleProperty();
   private final DoubleProperty xCoordHip = new SimpleDoubleProperty();

   double[] savedAngles = new double[4];
   private int comparisonFlag = 0;
   private int verificationMode = 0;
   private int savedCount = 0;

   SerialPort arduinoPort = null; // 
   ObservableList<String> portList; // list of all open COM ports

   StringBuilder angle1 = new StringBuilder();
   StringBuilder angle2 = new StringBuilder();
   StringBuilder angle3 = new StringBuilder();
   StringBuilder angle4 = new StringBuilder();

   StringBuilder savedAngle1 = new StringBuilder();
   StringBuilder savedAngle2 = new StringBuilder();
   StringBuilder savedAngle3 = new StringBuilder();
   StringBuilder savedAngle4 = new StringBuilder();

   Rotate rxBox = new Rotate(0, 0, 0, 0, Rotate.X_AXIS);
   Rotate ryBox = new Rotate(0, -100, 50, 0, Rotate.Y_AXIS);
   Rotate rzBox = new Rotate(0, -100, 50, 0, Rotate.Z_AXIS);

   Rotate ryBox2 = new Rotate(0, 0, 0, 0, Rotate.Y_AXIS);
   Rotate rzBox2 = new Rotate(0, -100, 0, 0, Rotate.Z_AXIS);

   Rotate cameraX = new Rotate(0, 0, 0, 0, Rotate.X_AXIS);
   Rotate cameraY = new Rotate(0, 0, 0, 0, Rotate.Y_AXIS);
   Rotate cameraZ = new Rotate(0, 0, 0, 0, Rotate.Z_AXIS);

   Translate translateKnee = new Translate();
   Translate translateHip = new Translate();

   Box thigh = new Box(200, 100, 100);
   Box shank = new Box(200, 100, 100);
   Box table = new Box(1100, 50, 400);
   Box torso = new Box(500, 100, 250);
   Box staticLeg = new Box(400, 100, 100);
   Sphere head = new Sphere(60);

   Group thighContainer = new Group(thigh);
   Group shankContainer = new Group(shank);
   Group tableContainer = new Group(table);
   Group staticLegContainer = new Group(staticLeg);
   Group torsoContainer = new Group(torso);
   Group headContainer = new Group(head);

   int count = 0;
   Boolean receivingMessage = false;

   final PhongMaterial tanMaterial = new PhongMaterial();
   final PhongMaterial tableMaterial = new PhongMaterial();

   Label intExtLabel = new Label("Hip External Rotation Angle: ");
   Label hipAbAdLabel = new Label("Hip Abduction Angle: ");
   Label kneeFlexExtenLabel = new Label("Knee Flexion Angle: ");
   Label hipFlexExtenLabel = new Label("Hip Flexion Angle: ");

   Label savedIntExtLabel = new Label("Saved Hip External Rotation Angle: ");
   Label savedHipAbAdLabel = new Label("Saved Hip Abduction Angle: ");
   Label savedKneeFlexExtenLabel = new Label("Saved Knee Flexion Angle: ");
   Label savedHipFlexExtenLabel = new Label("Saved Hip Flexion Angle: ");

   VBox ports = new VBox();

   @Override
   public void start(Stage primaryStage) {
      imv.setLayoutX(1400);
      imv.setLayoutY(500);

      PerspectiveCamera cameraAerialView = new PerspectiveCamera();
      Button aerialViewBtn = new Button();
      aerialViewBtn.setLayoutX(300);
      aerialViewBtn.setLayoutY(800);
      aerialViewBtn.setText("Change to top view");

      PerspectiveCamera cameraFootView = new PerspectiveCamera();
      Button footViewBtn = new Button();
      footViewBtn.setLayoutX(600);
      footViewBtn.setLayoutY(800);
      footViewBtn.setText("Change to front view");

      ParallelCamera cameraSideView = new ParallelCamera();
      Button sideViewBtn = new Button();
      sideViewBtn.setLayoutX(900);
      sideViewBtn.setLayoutY(800);
      sideViewBtn.setText("Change to side view");

      ParallelCamera cameraIsoView = new ParallelCamera();
      Button isoViewBtn = new Button();
      isoViewBtn.setLayoutX(1200);
      isoViewBtn.setLayoutY(800);
      isoViewBtn.setText("Change to isometric view");

      Button calibrateBtn = new Button();
      calibrateBtn.setLayoutX(200);
      calibrateBtn.setLayoutY(200);
      calibrateBtn.setText("Calibrate");

      Button snapshotBtn = new Button();
      snapshotBtn.setLayoutX(200);
      snapshotBtn.setLayoutY(250);
      snapshotBtn.setText("Take a snapshot");

      kneeAngle.set(0); // knee angle property
      hipFlexAngle.set(0); // hip adduction 
      hipAddAbAngle.set(0); // hip flex angle
      intRotAngle.set(0);
      xCoordKnee.set(0); // xcoord to move second box
      yCoordKnee.set(0); // ycoord to move second box
      xCoordHip.set(0);
      zCoordHip.set(0);

      rxBox.angleProperty().bind(intRotAngle);
      rzBox.angleProperty().bind(hipFlexAngle);
      ryBox.angleProperty().bind(hipAddAbAngle);
      rzBox2.angleProperty().bind(kneeAngle);
      translateKnee.xProperty().bind(xCoordKnee);
      translateKnee.yProperty().bind(yCoordKnee);
      translateHip.xProperty().bind(xCoordHip);
      translateHip.zProperty().bind(zCoordHip);

      moveContainers();
      moveLabels();

      thigh.getTransforms().addAll(rzBox, ryBox, rxBox);
      shank.getTransforms().addAll(rzBox, rzBox2, translateKnee, ryBox, translateHip, rxBox);

      tanMaterial.setDiffuseColor(Color.TAN);
      tableMaterial.setDiffuseColor(Color.DARKGRAY);
      thigh.setMaterial(tanMaterial);
      shank.setMaterial(tanMaterial);
      table.setMaterial(tableMaterial);
      staticLeg.setMaterial(tanMaterial);
      torso.setMaterial(tanMaterial);
      head.setMaterial(tanMaterial);

      detectPort();

      Group root = new Group();
      Group subroot = new Group();
      Scene scene = new Scene(root, 1920, 1080, true, SceneAntialiasing.BALANCED);
      SubScene subScene = new SubScene(subroot, 1920, 1080, true, SceneAntialiasing.BALANCED);

      root.getChildren().addAll(ports, isoViewBtn, aerialViewBtn, footViewBtn, calibrateBtn,
              sideViewBtn, intExtLabel, hipAbAdLabel, kneeFlexExtenLabel, hipFlexExtenLabel,
              savedIntExtLabel, savedHipAbAdLabel, savedKneeFlexExtenLabel, savedHipFlexExtenLabel,
              snapshotBtn, imv);
      subroot.getChildren().addAll(staticLegContainer, thighContainer,
              shankContainer, tableContainer, torsoContainer, headContainer);
      root.getChildren().add(subScene);

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

      // initialize the viewing angle to aerial view
      cameraAerialView.getTransforms().removeAll(cameraX, cameraY, cameraZ);
      cameraAerialView.setTranslateY(-200);
      cameraX.setAngle(-80);
      cameraAerialView.setTranslateZ(300);
      cameraAerialView.getTransforms().addAll(cameraX);
      subScene.setCamera(cameraAerialView);

      primaryStage.setTitle("Hip Position Indicator");
      primaryStage.setMaximized(true);
      primaryStage.setScene(scene);

      primaryStage.show();

      // eventlistener for the calibration popup functionality
      calibrateBtn.setOnAction(new EventHandler<ActionEvent>() {
         @Override
         public void handle(ActionEvent event) {
            try {
               FXMLLoader fxmlLoader = new FXMLLoader(getClass().getResource("Calibration.fxml"));
               Parent root1 = (Parent) fxmlLoader.load();
               Stage stage = new Stage();
               stage.initModality(Modality.APPLICATION_MODAL);
               stage.setTitle("Calibrating...");
               stage.setScene(new Scene(root1));
               stage.show();
            } 
            catch (IOException e) {
               e.printStackTrace();
            }
         }
      });

      // eventlistener for the aerial camera 
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

      // eventlistener for front camera view
      footViewBtn.setOnAction(new EventHandler<ActionEvent>() {
         @Override
         public void handle(ActionEvent event) {
            //camera.setTranslateX(-400);
            cameraFootView.getTransforms().removeAll(cameraY, cameraX, cameraZ);
            cameraFootView.setTranslateX(900);
            cameraY.setAngle(-90);
            cameraX.setAngle(-10);

            cameraFootView.setTranslateZ(-700);
            cameraFootView.getTransforms().addAll(cameraY, cameraX);
            subScene.setCamera(cameraFootView);
         }
      });

      // eventlistener for side view
      sideViewBtn.setOnAction(new EventHandler<ActionEvent>() {
         @Override
         public void handle(ActionEvent event) {
            //camera.setTranslateX(-400);
            cameraSideView.getTransforms().removeAll(cameraY, cameraX, cameraZ);
            cameraX.setAngle(0);
            cameraY.setAngle(-180);
            cameraZ.setAngle(0);
            cameraSideView.setTranslateX(1700);
            cameraSideView.setTranslateY(0);
            cameraSideView.setTranslateZ(-500);
            cameraSideView.getTransforms().addAll(cameraX, cameraY, cameraZ);
            subScene.setCamera(cameraSideView);
         }
      });

      // eventlistener to change camera angle 
      isoViewBtn.setOnAction(new EventHandler<ActionEvent>() {
         @Override
         public void handle(ActionEvent event) {
            cameraIsoView.getTransforms().removeAll(cameraY, cameraX, cameraZ);
            cameraY.setAngle(-120);
            cameraX.setAngle(20);
            cameraZ.setAngle(20);
            cameraIsoView.setTranslateZ(-800);
            cameraIsoView.setTranslateX(900);
            cameraIsoView.setTranslateY(1);
            cameraIsoView.getTransforms().addAll(cameraX, cameraY, cameraZ);
            subScene.setCamera(cameraIsoView);
         }
      });

      // eventlistener for snapshot functionality
      snapshotBtn.setOnAction(new EventHandler<ActionEvent>() {
         @Override
         public void handle(ActionEvent event) {
            WritableImage image = subScene.snapshot(new SnapshotParameters(), null);

            // TODO: probably use a file chooser here
            File file = new File("savedPosition.png");

            try {
               ImageIO.write(SwingFXUtils.fromFXImage(image, null), "png", file);
               Image img = new Image("file:savedPosition.png", 600, 480, true, true);
               imv.setImage(img);
            } 
            catch (IOException e) {
               e.printStackTrace();
            }
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

   /* 
      Reads in every byte sent in the serial port. If a \n is found, then 
      package that string and convert it to the int to make to angle. If a comma 
      is found, up the count and append the to the next angle. Endless stream 
      until the application is shut down. 
   */
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
                  byte buffer[] = serialPort.readBytes(serialPortEvent.getEventValue());
                  for (byte b : buffer) {
                     if (b == 118) {
                        verificationMode = 1;
                        comparisonFlag = 1;
                     } 
                     else {
                        if (verificationMode == 1) {
                           if (b == 44) {
                              savedCount++;
                           } 
                           else if (b == 10) {
                              savedCount = 0;
                              verificationMode = 0;

                              String angle1Process = savedAngle1.toString();
                              String angle2Process = savedAngle2.toString();
                              String angle3Process = savedAngle3.toString();
                              String angle4Process = savedAngle4.toString();
                              
                              String temp1 = new String();
                              temp1 = angle1Process.trim();

                              String temp2 = new String();
                              temp2 = angle2Process.trim();
                              
                              String temp3 = new String();
                              temp3 = angle3Process.trim();
                              
                              String temp4 = new String();
                              temp4 = angle4Process.trim();

                              System.out.println("Saved Angle1: " + angle1Process);
                              System.out.println("Saved Angle2: " + angle2Process);
                              System.out.println("Saved Angle3: " + angle3Process);
                              System.out.println("Saved Angle4: " + angle4Process);
                              try {

                                 savedAngle1.setLength(0);
                                 savedAngle2.setLength(0);
                                 savedAngle3.setLength(0);
                                 savedAngle4.setLength(0);

                                 // first angle calculations
                                 savedAngles[0] = Double.parseDouble(temp1);
                                 Platform.runLater(() -> {
                                    savedIntExtLabel.setText("Saved Hip External Rotation Angle: " + -savedAngles[0]);
                                 });

                                 // second angle calculations
                                 savedAngles[1] = Double.parseDouble(temp2);
                                 Platform.runLater(() -> {
                                    savedHipAbAdLabel.setText("Saved Hip Abduction Angle: " + -savedAngles[1]);
                                 });

                                 // third angle calculations
                                 savedAngles[2] = Double.parseDouble(temp3);
                                 Platform.runLater(() -> {
                                    savedKneeFlexExtenLabel.setText("Saved Knee Flexion Angle: " + savedAngles[2]);
                                 });

                                 // last angle calculations
                                 savedAngles[3] = Double.parseDouble(temp4);
                                 Platform.runLater(() -> {
                                    savedHipFlexExtenLabel.setText("Saved Hip Flexion Angle: " + -savedAngles[3]);
                                 });

                              } catch (NumberFormatException e) {
                                 e.printStackTrace();
                              }
                           } 
                           else {
                              if (savedCount == 0) {
                                 savedAngle1.append((char) b);
                              } 
                              else if (savedCount == 1) {
                                 savedAngle2.append((char) b);
                              } 
                              else if (savedCount == 2) {
                                 savedAngle3.append((char) b);
                              } 
                              else if (savedCount == 3) {
                                 savedAngle4.append((char) b);
                              }
                           }
                        } 
                        else if (verificationMode == 0) {
                           if (b == 44) {
                              count++;
                           } 
                           else if (b == 10) {

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

                                 // external angle calculations
                                 Integer x = Integer.parseInt(temp1);
                                 Platform.runLater(() -> {
                                    intExtLabel.setText("Hip External Rotation Angle: " + -x);
                                 });
                                 intRotAngle.set(x);

                                 // hip abduction angle calculations
                                 Integer y = Integer.parseInt(temp2);
                                 Platform.runLater(() -> {
                                    hipAbAdLabel.setText("Hip Abduction Angle: " + -y);
                                 });
                                 hipAddAbAngle.set(y);
                                 Platform.runLater(() -> {
                                    shank.getTransforms().removeAll(ryBox2, rzBox2);
                                 });

                                 zCoordHip.set(200 * Math.sin(-hipAddAbAngle.get() * (Math.PI / 180)));
                                 xCoordHip.set(200 - (200 * Math.cos(-hipAddAbAngle.get() * (Math.PI / 180))));
                                 Platform.runLater(() -> {
                                    shank.getTransforms().addAll(ryBox2, rzBox2);
                                 });

                                 // knee flexion angle calculations
                                 Integer z = Integer.parseInt(temp3);

                                 if (z >= 0) {
                                    kneeAngle.set(z);
                                    Platform.runLater(() -> {
                                       kneeFlexExtenLabel.setText("Knee Flexion Angle: " + z);
                                    });
                                 }

                                 // last angle calculations
                                 Integer a = Integer.parseInt(temp4);
                                 Platform.runLater(() -> {
                                    hipFlexExtenLabel.setText("Hip Flexion Angle: " + -a);
                                 });
                                 Platform.runLater(() -> {
                                    shank.getTransforms().removeAll(rzBox2, ryBox2);
                                 });

                                 hipFlexAngle.set(a);

                                 xCoordKnee.set(200 - (200 * Math.cos(-hipFlexAngle.get() * (Math.PI / 180))));
                                 yCoordKnee.set(-200 * Math.sin(-hipFlexAngle.get() * (Math.PI / 180)));
                                 Platform.runLater(() -> {
                                    shank.getTransforms().addAll(rzBox2, ryBox2);
                                 });

                                 if (comparisonFlag != 0) {
                                    // int rot angle
                                    if (returnColorLabel(savedAngles[0], x) == 1) {
                                       // int ext angle red
                                       savedIntExtLabel.setTextFill(Color.RED);
                                    } else if (returnColorLabel(savedAngles[0], x) == 2) {
                                       // int ext angle green
                                       savedIntExtLabel.setTextFill(Color.GREEN);
                                    } else if (returnColorLabel(savedAngles[0], x) == 3) {
                                       // int ext angle yellow
                                       savedIntExtLabel.setTextFill(Color.web("#b2ad1c"));
                                    }

                                    // hip add ab angle
                                    if (returnColorLabel(savedAngles[1], y) == 1) {
                                       // hip Ad ab angle red
                                       savedHipAbAdLabel.setTextFill(Color.RED);
                                    } else if (returnColorLabel(savedAngles[1], y) == 2) {
                                       // hip ad ab angle green
                                       savedHipAbAdLabel.setTextFill(Color.GREEN);
                                    } else if (returnColorLabel(savedAngles[1], y) == 3) {
                                       // hip ad ab angle yellow
                                       savedHipAbAdLabel.setTextFill(Color.web("#b2ad1c"));
                                    }

                                    // knee flex exten angle
                                    if (returnColorLabel(savedAngles[2], z) == 1) {
                                       // hip Ad ab angle red
                                       savedKneeFlexExtenLabel.setTextFill(Color.RED);
                                    } else if (returnColorLabel(savedAngles[2], z) == 2) {
                                       // hip ad ab angle green
                                       savedKneeFlexExtenLabel.setTextFill(Color.GREEN);
                                    } else if (returnColorLabel(savedAngles[2], z) == 3) {
                                       // hip ad ab angle yellow
                                       savedKneeFlexExtenLabel.setTextFill(Color.web("#b2ad1c"));
                                    }

                                    // hip flex exten angle
                                    if (returnColorLabel(savedAngles[3], a) == 1) {
                                       // hip Ad ab angle red
                                       savedHipFlexExtenLabel.setTextFill(Color.RED);
                                    } else if (returnColorLabel(savedAngles[3], a) == 2) {
                                       // hip ad ab angle green
                                       savedHipFlexExtenLabel.setTextFill(Color.GREEN);
                                    } else if (returnColorLabel(savedAngles[3], a) == 3) {
                                       // hip ad ab angle yellow
                                       savedHipFlexExtenLabel.setTextFill(Color.web("#b2ad1c"));
                                    }
                                 }
                              } 
                              catch (NumberFormatException e) {
                                 e.printStackTrace();
                              }

                           } 
                           else {
                              if (count == 0) {
                                 angle1.append((char) b);
                              } 
                              else if (count == 1) {
                                 angle2.append((char) b);
                              } 
                              else if (count == 2) {
                                 angle3.append((char) b);
                              } 
                              else if (count == 3) {
                                 angle4.append((char) b);
                              }
                           }
                        }
                     }
                  }
               } 
               catch (SerialPortException ex) {
                  Logger.getLogger(Hps.class.getName())
                   .log(Level.SEVERE, null, ex);
               } 
               catch (NumberFormatException e) {
                  e.printStackTrace();
               }
            }
         });

         arduinoPort = serialPort;
         success = true;
      } 
      catch (SerialPortException ex) {
         Logger.getLogger(Hps.class.getName()).log(Level.SEVERE, null, ex);
         System.out.println("SerialPortException: " + ex.toString());
      }
      return success;
   }

   public int returnColorLabel(double sAngle, int lAngle) {
      if (Math.abs(sAngle - lAngle) > 10) {
         return 1;
      }
      if (Math.abs(sAngle - lAngle) <= 3) {
         return 2;
      }
      if (Math.abs(sAngle - lAngle) <= 10 && Math.abs(sAngle - lAngle) > 3) {
         return 3;
      }
      return 0;
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

         } 
         catch (SerialPortException ex) {
            Logger.getLogger(Hps.class.getName())
                    .log(Level.SEVERE, null, ex);
         }
      }
   }

   public void stop() throws Exception {
      disconnectArduino();
   }
   
   public void moveLabels() {
      intExtLabel.setLayoutX(1400);
      intExtLabel.setLayoutY(160);
      intExtLabel.setFont(Font.font("Arial", 24));

      hipAbAdLabel.setLayoutX(1400);
      hipAbAdLabel.setLayoutY(200);
      hipAbAdLabel.setFont(Font.font("Arial", 24));

      hipFlexExtenLabel.setLayoutX(1400);
      hipFlexExtenLabel.setLayoutY(240);
      hipFlexExtenLabel.setFont(Font.font("Arial", 24));

      kneeFlexExtenLabel.setLayoutX(1400);
      kneeFlexExtenLabel.setLayoutY(280);
      kneeFlexExtenLabel.setFont(Font.font("Arial", 24));

      savedIntExtLabel.setLayoutX(1400);
      savedIntExtLabel.setLayoutY(350);
      savedIntExtLabel.setFont(Font.font("Arial", 24));

      savedHipAbAdLabel.setLayoutX(1400);
      savedHipAbAdLabel.setLayoutY(390);
      savedHipAbAdLabel.setFont(Font.font("Arial", 24));

      savedHipFlexExtenLabel.setLayoutX(1400);
      savedHipFlexExtenLabel.setLayoutY(430);
      savedHipFlexExtenLabel.setFont(Font.font("Arial", 24));

      savedKneeFlexExtenLabel.setLayoutX(1400);
      savedKneeFlexExtenLabel.setLayoutY(470);
      savedKneeFlexExtenLabel.setFont(Font.font("Arial", 24));
   }

   public void moveContainers() {
      thighContainer.setTranslateX(1100);
      shankContainer.setTranslateX(1300);
      thighContainer.setTranslateY(500);
      shankContainer.setTranslateY(500);
      thighContainer.setTranslateZ(150);
      shankContainer.setTranslateZ(150);
      tableContainer.setTranslateX(900);
      tableContainer.setTranslateY(575);
      tableContainer.setTranslateZ(100);
      staticLegContainer.setTranslateX(1200);
      staticLegContainer.setTranslateY(500);
      torsoContainer.setTranslateX(760);
      torsoContainer.setTranslateY(500);
      torsoContainer.setTranslateZ(75);
      headContainer.setTranslateX(450);
      headContainer.setTranslateY(500);
      headContainer.setTranslateZ(60);
   }

   /**
    * @param args the command line arguments
    */
   public static void main(String[] args) {
      launch(args);
   }
}
