#include <noether_ros/located_vector_generator_widgets.h>

#include <noether_tpp/serialization.h>
#include <QLineEdit>
#include <QLabel>
#include <QFormLayout>

static const char* SERVICE_NAME_KEY = "service_name";
static const char* DEFAULT_SERVICE_NAME = "get_located_vector";

namespace noether_ros
{
LocatedVectorDirectionGeneratorWidget::LocatedVectorDirectionGeneratorWidget(QWidget* parent)
  : BaseWidget(parent), line_edit_service_name_(new QLineEdit(this))
{
  line_edit_service_name_->setPlaceholderText(DEFAULT_SERVICE_NAME);
  line_edit_service_name_->setText(DEFAULT_SERVICE_NAME);
  line_edit_service_name_->setToolTip("Name of the service providing noether_ros/srv/GetLocatedVector");

  auto layout = new QFormLayout(this);
  layout->addRow("Service name", line_edit_service_name_);
}

void LocatedVectorDirectionGeneratorWidget::configure(const YAML::Node& config)
{
  if (config[SERVICE_NAME_KEY])
    line_edit_service_name_->setText(QString::fromStdString(YAML::getMember<std::string>(config, SERVICE_NAME_KEY)));
}

void LocatedVectorDirectionGeneratorWidget::save(YAML::Node& config) const
{
  config["name"] = "LocatedVectorDirection";
  config[SERVICE_NAME_KEY] = line_edit_service_name_->text().toStdString();
}

LocatedVectorOriginGeneratorWidget::LocatedVectorOriginGeneratorWidget(QWidget* parent)
  : BaseWidget(parent), line_edit_service_name_(new QLineEdit(this))
{
  line_edit_service_name_->setPlaceholderText(DEFAULT_SERVICE_NAME);
  line_edit_service_name_->setText(DEFAULT_SERVICE_NAME);
  line_edit_service_name_->setToolTip("Name of the service providing noether_ros/srv/GetLocatedVector");

  auto layout = new QFormLayout(this);
  layout->addRow("Service name", line_edit_service_name_);
}

void LocatedVectorOriginGeneratorWidget::configure(const YAML::Node& config)
{
  if (config[SERVICE_NAME_KEY])
    line_edit_service_name_->setText(QString::fromStdString(YAML::getMember<std::string>(config, SERVICE_NAME_KEY)));
}

void LocatedVectorOriginGeneratorWidget::save(YAML::Node& config) const
{
  config["name"] = "LocatedVectorOrigin";
  config[SERVICE_NAME_KEY] = line_edit_service_name_->text().toStdString();
}

}  // namespace noether_ros
