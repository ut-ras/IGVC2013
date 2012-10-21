
(cl:in-package :asdf)

(defsystem "adc_sub20-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :std_msgs-msg
)
  :components ((:file "_package")
    (:file "sub20_ADC_err" :depends-on ("_package_sub20_ADC_err"))
    (:file "_package_sub20_ADC_err" :depends-on ("_package"))
    (:file "sub20_ADC_meas" :depends-on ("_package_sub20_ADC_meas"))
    (:file "_package_sub20_ADC_meas" :depends-on ("_package"))
  ))