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
        // 调用 LoRe 函数并显示结果
        std::string result = lore(file_path.toStdString());
        ui->resultTextEdit->setText(QString::fromStdString(result));
    } else {
        ui->resultTextEdit->setText("文件格式不正确");
    }
}

// 保存结果为TXT文件的槽函数
void MainWindow::saveResultToFile() {
    // 获取 resultTextEdit 中的文本
    QString resultText = ui->resultTextEdit->toPlainText();

    if (resultText.isEmpty()) {
        QMessageBox::warning(this, "保存失败", "结果文本为空，无法保存！");
        return;
    }

    // 打开文件保存对话框，用户选择保存路径
    QString fileName = QFileDialog::getSaveFileName(this, "保存结果为TXT", "", "Text Files (*.txt)");

    // 如果用户取消保存，返回
    if (fileName.isEmpty()) {
        return;
    }

    // 创建文件对象并尝试打开文件
    QFile file(fileName);
    if (!file.open(QIODevice::WriteOnly | QIODevice::Text)) {
        QMessageBox::warning(this, "保存失败", "无法打开文件进行写入！");
        return;
    }

    // 使用 QTextStream 写入文件
    QTextStream out(&file);
    out << resultText;
    file.close();

    // 提示用户保存成功
    QMessageBox::information(this, "保存成功", "结果已成功保存为TXT文件！");
}
