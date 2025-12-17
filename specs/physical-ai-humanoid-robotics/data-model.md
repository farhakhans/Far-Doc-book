# Data Model: Physical AI & Humanoid Robotics Book

## Content Entities

### Book Section
- **name**: String (required) - Title of the section
- **slug**: String (required) - URL-friendly identifier
- **content**: Markdown/MDX (required) - Main content
- **order**: Integer (required) - Display order in navigation
- **prerequisites**: Array[String] - Other sections that should be read first
- **learningOutcomes**: Array[String] - What students should learn from this section
- **assessments**: Array[AssessmentRef] - Associated assessment projects
- **codeExamples**: Array[CodeExample] - Embedded code examples
- **diagrams**: Array[Diagram] - Visual aids and diagrams

### Module
- **extends**: Book Section
- **moduleNumber**: Integer (required) - Module identifier (1-4)
- **duration**: String (required) - Expected time to complete (e.g., "2 weeks")
- **topics**: Array[String] - Specific topics covered
- **tools**: Array[String] - Tools and technologies used
- **practicalExercises**: Array[Exercise] - Hands-on activities

### Weekly Breakdown
- **weekNumber**: Integer (required) - Week identifier (1-13)
- **topics**: Array[String] - Topics for the week
- **readings**: Array[SectionRef] - Required reading materials
- **assignments**: Array[Assignment] - Weekly assignments
- **objectives**: Array[String] - Learning objectives for the week

### Code Example
- **title**: String (required) - Descriptive title
- **language**: String (required) - Programming language
- **code**: String (required) - The actual code
- **description**: String - Explanation of the code
- **fileLocation**: String - Where the code should be saved
- **dependencies**: Array[String] - Required dependencies

### Assessment Project
- **title**: String (required) - Project title
- **module**: ModuleRef (required) - Associated module
- **description**: String (required) - Detailed description
- **requirements**: Array[String] - Technical requirements
- **evaluationCriteria**: Array[String] - How the project will be graded
- **deliverables**: Array[String] - What students need to submit
- **estimatedTime**: String - Time needed to complete

### Hardware Specification
- **name**: String (required) - Hardware component name
- **type**: String (required) - Category (e.g., "workstation", "sensor", "robot")
- **specs**: Object - Technical specifications
- **cost**: String - Estimated cost
- **alternatives**: Array[HardwareRef] - Alternative options
- **compatibility**: Array[String] - Compatible systems
- **useCase**: String - Primary use case

### Learning Outcome
- **id**: String (required) - Unique identifier (e.g., "LO-001")
- **description**: String (required) - What the student should be able to do
- **module**: ModuleRef - Associated module
- **assessment**: AssessmentRef - How it's assessed
- **complexity**: String - Basic/Intermediate/Advanced

## Relationships

### Book Structure
- Book has many Book Sections
- Book Section has many Code Examples and Diagrams
- Module extends Book Section
- Module has many Practical Exercises
- Weekly Breakdown connects to multiple Book Sections

### Assessment Relationships
- Module has many Assessment Projects
- Learning Outcome connects to Module and Assessment Project
- Assessment Project has many Deliverables

### Hardware Relationships
- Hardware Specification may have many Alternatives
- Hardware Specification connects to Compatibility requirements

## State Transitions (if applicable)

### Content Review Process
- Draft → In Review → Approved → Published
- Published → Needs Update → In Review → Approved → Published

### Assessment Status
- Assigned → In Progress → Submitted → Graded → Complete

## Validation Rules

### Content Requirements
- All Book Sections must have a title and content
- All Modules must have a module number and duration
- All Code Examples must have a language specified
- All Learning Outcomes must connect to a module
- All Assessments must have requirements and evaluation criteria

### Structural Requirements
- Module numbers must be between 1 and 4
- Week numbers must be between 1 and 13
- Content order must not have duplicates
- Prerequisites must reference existing sections
- All file locations must be valid paths