#include "../include/mainwindow.h"
#include "../ui/ui_mainwindow.h"

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::MainWindow)
{
    ui->setupUi(this);
    connect(ui->saveButton, &QPushButton::clicked, this, &MainWindow::saveResultToFile);
}

MainWindow::~MainWindow()
{
    delete ui;
}

void MainWindow::dragEnterEvent(QDragEnterEvent *event) {
    if (event->mimeData()->hasUrls()) {
        event->acceptProposedAction();
    }
}

void MainWindow::dropEvent(QDropEvent *event) {
    QList<QUrl> urls = event->mimeData()->urls();
    if (urls.isEmpty()) {
        return;
    }
    QString file_path = urls.first().toLocalFile();
    if (file_path.endsWith(".pcd")) {
        // The LoRe function is called and the result is displayed
        std::string result = lore(file_path.toStdString());
        ui->resultTextEdit->setText(QString::fromStdString(result));
    } else {
        ui->resultTextEdit->setText("The file format is incorrect");
    }
}

// Save the result as a slot function for a TXT file
void MainWindow::saveResultToFile() {
    // 获取 resultTextEdit 中的文本
    QString resultText = ui->resultTextEdit->toPlainText();

    if (resultText.isEmpty()) {
        QMessageBox::warning(this, "Save failed", "The resulting text is empty and cannot be saved!");
        return;
    }

    // The file save dialog box opens, and the user selects the save path
    QString fileName = QFileDialog::getSaveFileName(this, "The saved result is TXT", "", "Text Files (*.txt)");

    // If the user cancels the save, it returns
    if (fileName.isEmpty()) {
        return;
    }

    // 创建文件对象并尝试打开文件
    QFile file(fileName);
    if (!file.open(QIODevice::WriteOnly | QIODevice::Text)) {
        QMessageBox::warning(this, "Save failed", "Unable to open file for writing！");
        return;
    }

    // 使用 QTextStream 写入文件
    QTextStream out(&file);
    out << resultText;
    file.close();

    // 提示用户保存成功
    QMessageBox::information(this, "Save success", "The result has been successfully saved as a TXT file！");
}

void MainWindow::mousePressEvent(QMouseEvent *event) {
    // 检查是否点击在 dragDropArea
    if (ui->dragDropArea->geometry().contains(event->pos())) {
        // 打开文件选择对话框，用户选择 PCD 文件
        QString fileName = QFileDialog::getOpenFileName(this, "Select the PCD file", "", "PCD Files (*.pcd)");

        if (!fileName.isEmpty()) {
            // 调用 LoRe 函数处理文件并显示结果
            std::string result = lore(fileName.toStdString());
            ui->resultTextEdit->setText(QString::fromStdString(result));
        } else {
            ui->resultTextEdit->setText("No files are selected");
        }
    }
    ui->dragDropArea->setStyleSheet("background-color: #e0e0e0; border: 2px solid #888;");
}
