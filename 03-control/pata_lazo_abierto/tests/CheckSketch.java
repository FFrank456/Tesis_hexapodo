import java.nio.file.*;
import java.io.*;
import processing.mode.java.preproc.*;
class CheckSketch {
  public static void main(String[] args) throws Exception {
    processing.app.Preferences.skipInit();
    Path source=Path.of(args[0]);
    String name=source.getFileName().toString().replace(".pde", "");
    Files.createDirectories(Path.of(args[1]));
    try (Writer out=Files.newBufferedWriter(Path.of(args[1],name+".java"))) {
      PdePreprocessor.builderFor(name).setTabSize(2).setIsTesting(true).build().write(out,Files.readString(source));
    }
  }
}
